//! MCU Command Framework — packet structs, send/wait, command catalog
//!
//! Provides McuTxd/UniTxd packet encoding, IRQ-based command waiting,
//! and named command functions for firmware/radio initialization.

use userlib::{uerror, udebug, ulog};
use userlib::ipc::{Irq, Mux, MuxFilter};
use crate::regs::*;
use crate::device::{Mt76Device, DeviceError};
use crate::dma::TxRing;

// ============================================================================
// Error types
// ============================================================================

#[derive(Debug)]
pub enum McuError {
    TxTimeout { cmd: &'static str },
    RxTimeout { cmd: &'static str, seq: u8 },
    Rejected { cmd: &'static str, status: u32 },
    Device(DeviceError),
    RingFull,
}

impl McuError {
    pub fn name(&self) -> &'static str {
        match self {
            Self::TxTimeout { .. } => "mcu_tx_timeout",
            Self::RxTimeout { .. } => "mcu_rx_timeout",
            Self::Rejected { .. } => "mcu_rejected",
            Self::Device(_) => "device_error",
            Self::RingFull => "ring_full",
        }
    }
}

impl core::fmt::Display for McuError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::TxTimeout { cmd } => write!(f, "MCU TX timeout: {}", cmd),
            Self::RxTimeout { cmd, seq } => write!(f, "MCU RX timeout: {} seq={}", cmd, seq),
            Self::Rejected { cmd, status } => write!(f, "MCU rejected: {} status={}", cmd, status),
            Self::Device(e) => write!(f, "MCU device error: {}", e),
            Self::RingFull => write!(f, "MCU ring full"),
        }
    }
}

impl From<DeviceError> for McuError {
    fn from(e: DeviceError) -> Self { Self::Device(e) }
}

// ============================================================================
// IRQ waiter for firmware loading / MCU commands
// ============================================================================

/// IRQ waiter with a Mux for blocking on WFDMA interrupts.
pub struct FwIrq {
    pub irq: Irq,
    mux: Mux,
}

impl FwIrq {
    pub fn new(irq: Irq) -> Result<Self, McuError> {
        let mux = Mux::new().map_err(|_| McuError::RingFull)?;
        mux.add(irq.handle(), MuxFilter::Readable).map_err(|_| McuError::RingFull)?;
        let _ = mux.set_timeout(20);
        Ok(Self { irq, mux })
    }

    pub fn wait(&self) -> bool {
        self.mux.wait().is_ok()
    }

    pub fn ack(&mut self) {
        let _ = self.irq.ack();
    }

    pub fn into_irq(self) -> Irq {
        self.irq
    }
}

// ============================================================================
// Packet constants
// ============================================================================

const MCU_PKT_ID: u8 = 0xa0;

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

pub mod mcu_q {
    pub const SET: u8 = 1;
    #[allow(dead_code)]
    pub const NA: u8 = 3;
}

pub mod dl_mode {
    pub const ENCRYPT: u32 = 1 << 0;
    pub const RESET_SEC_IV: u32 = 1 << 3;
    pub const WORKING_PDA_CR4: u32 = 1 << 4;
    pub const NEED_RSP: u32 = 1u32 << 31;
}

pub mod fw_feature {
    pub const SET_ENCRYPT: u8 = 1 << 0;
    pub const SET_KEY_IDX_MASK: u8 = 0x6;
    pub const OVERRIDE_ADDR: u8 = 1 << 5;
}

pub mod fw_start {
    pub const OVERRIDE: u32 = 1 << 0;
    pub const WORKING_PDA_CR4: u32 = 1 << 2;
    pub const WORKING_PDA_DSP: u32 = 1 << 3;
}

const S2D_H2N: u8 = 0;
const S2D_H2C: u8 = 2;
const S2D_H2CN: u8 = 3;

const MCU_CMD_ACK: u8 = 1 << 0;
const MCU_CMD_UNI: u8 = 1 << 1;
const MCU_CMD_SET: u8 = 1 << 2;

const CMD_FIELD_QUERY: u32 = 1 << 16;
const CMD_FIELD_UNI: u32 = 1 << 17;
const CMD_FIELD_WA: u32 = 1 << 19;
const CMD_FIELD_WM: u32 = 1 << 20;

// UNI command IDs used by mcu_uni_cmd()
pub const MCU_UNI_CMD_WSYS_CONFIG: u16 = 0x0b;
pub const MCU_UNI_CMD_EFUSE_CTRL: u16 = 0x2d;
pub const MCU_UNI_CMD_VOW: u16 = 0x37;
pub const MCU_UNI_CMD_RRO: u16 = 0x57;
pub const MCU_UNI_CMD_SDO: u16 = 0x88;

// ============================================================================
// MCU packet structs
// ============================================================================

/// Legacy MCU TXD — firmware download and WA commands.
/// 64 bytes total.
#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct McuTxd {
    txd: [u32; 8],
    len: u16,
    pq_id: u16,
    cid: u8,
    pkt_type: u8,
    set_query: u8,
    seq: u8,
    uc_d2b0_rev: u8,
    ext_cid: u8,
    s2d_index: u8,
    ext_cid_ack: u8,
    rsv: [u32; 5],
}

impl McuTxd {
    const SIZE: usize = 64;

    fn new(cmd: u8, payload_len: usize, seq: u8) -> Self {
        let mut txd = Self::default();
        let total = Self::SIZE + payload_len;
        txd.txd[0] = (total as u32 & 0xffff) | (2 << 23) | (0x20 << 25);
        txd.txd[1] = 1 << 14; // HDR_FORMAT_CMD
        txd.len = ((total - 32) as u16).to_le();
        txd.pq_id = 0x8000u16.to_le();
        txd.cid = cmd;
        txd.pkt_type = MCU_PKT_ID;
        txd.set_query = mcu_q::NA;
        txd.seq = seq;
        txd.s2d_index = S2D_H2N;
        txd
    }

    /// Build legacy MCU TXD with ext_cid (for WA EXT commands like MWDS)
    fn new_ext(cid: u8, ext_cid: u8, payload_len: usize, seq: u8, s2d: u8) -> Self {
        let mut txd = Self::new(cid, payload_len, seq);
        txd.ext_cid = ext_cid;
        txd.s2d_index = s2d;
        txd.set_query = mcu_q::SET;
        txd.ext_cid_ack = 1;
        txd
    }

    fn as_bytes(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self as *const _ as *const u8, Self::SIZE) }
    }
}

/// UNI MCU TXD — post-firmware commands.
/// 48 bytes total.
#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct UniTxd {
    txd: [u32; 8],
    len: u16,
    cid: u16,
    rsv: u8,
    pkt_type: u8,
    frag_n: u8,
    seq: u8,
    checksum: u16,
    s2d_index: u8,
    option: u8,
    rsv1: [u8; 4],
}

impl UniTxd {
    const SIZE: usize = 48;

    fn as_bytes(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self as *const _ as *const u8, Self::SIZE) }
    }
}

// ============================================================================
// Buffer construction helpers
// ============================================================================

/// Write a TLV header (tag + len) at the given offset. Returns offset past the header (off + 4).
#[inline]
fn write_tlv(buf: &mut [u8], off: usize, tag: u16, len: u16) -> usize {
    buf[off..off + 2].copy_from_slice(&tag.to_le_bytes());
    buf[off + 2..off + 4].copy_from_slice(&len.to_le_bytes());
    off + 4
}

/// Write the 8-byte STA request header at offset 0.
/// Layout: bss_idx(1) + wlan_idx_lo(1) + tlv_num(2) + is_tlv_append(1) + muar_idx(1) + wlan_idx_hi(1) + rsv(1).
fn write_sta_hdr(buf: &mut [u8], bss_idx: u8, wlan_idx: u16, omac_idx: u8, tlv_num: u16) {
    buf[0] = bss_idx;
    buf[1] = wlan_idx as u8;
    buf[2..4].copy_from_slice(&tlv_num.to_le_bytes());
    buf[4] = 1; // is_tlv_append
    buf[5] = omac_idx;
    buf[6] = (wlan_idx >> 8) as u8;
}

// ============================================================================
// Send helpers
// ============================================================================

/// Send a legacy MCU command (McuTxd format) on a TX ring.
pub fn send_cmd(
    dev: &Mt76Device, ring: &mut TxRing,
    cmd: u8, data: &[u8], seq: u8,
) -> Result<(), McuError> {
    let txd = McuTxd::new(cmd, data.len(), seq);
    ring.submit_cmd(dev, txd.as_bytes(), data);
    clear_pending_interrupts(dev);
    Ok(())
}

/// Send a legacy MCU command with ext_cid (for WA EXT commands like MWDS).
pub fn send_cmd_ext(
    dev: &Mt76Device, ring: &mut TxRing,
    cid: u8, ext_cid: u8, s2d: u8, data: &[u8], seq: u8,
) -> Result<(), McuError> {
    let txd = McuTxd::new_ext(cid, ext_cid, data.len(), seq, s2d);
    ring.submit_cmd(dev, txd.as_bytes(), data);
    clear_pending_interrupts(dev);
    Ok(())
}

/// Send a UNI MCU command with optional response waiting.
pub fn send_uni_cmd(
    dev: &Mt76Device, ring: &mut TxRing,
    cmd: u32, data: &[u8], wait: bool, seq: u8,
    irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let mcu_cmd = (cmd & 0xFF) as u16;
    let total = UniTxd::SIZE + data.len();

    let mut uni = UniTxd::default();
    uni.txd[0] = (total as u32 & 0xffff) | (2 << 23) | (0x20 << 25);
    uni.txd[1] = 1 << 14;
    uni.len = ((total - 32) as u16).to_le();
    uni.cid = mcu_cmd.to_le();
    uni.pkt_type = MCU_PKT_ID;
    uni.seq = seq;

    // Routing
    uni.s2d_index = S2D_H2CN;
    if (cmd & CMD_FIELD_WA) != 0 && (cmd & CMD_FIELD_WM) == 0 {
        uni.s2d_index = S2D_H2C;
    } else if (cmd & CMD_FIELD_WM) != 0 && (cmd & CMD_FIELD_WA) == 0 {
        uni.s2d_index = S2D_H2N;
    }

    // Option flags
    if (cmd & CMD_FIELD_QUERY) != 0 {
        uni.option = MCU_CMD_ACK | MCU_CMD_UNI;
    } else {
        uni.option = MCU_CMD_ACK | MCU_CMD_UNI | MCU_CMD_SET;
    }
    if mcu_cmd == MCU_UNI_CMD_SDO {
        uni.option &= !MCU_CMD_ACK;
    }

    let rx_snap = if wait { dev.rr(ring.rx_regs + MT_QUEUE_DMA_IDX) } else { 0 };

    ring.submit_cmd(dev, uni.as_bytes(), data);
    if irq.is_some() {
        clear_pending_interrupts(dev);
    }

    wait_tx_done(dev, ring, ring.last_submitted(), 2000, "uni_cmd")?;

    // Wait for RX response
    if wait {
        let result = wait_rx_response(dev, ring, rx_snap, seq, 5000, irq, "uni_cmd");
        if result.is_err() {
            uerror!("mcu", "uni_cmd_failed"; mcu_cmd = mcu_cmd as u32, seq = seq as u32);
        }
        result?;
    }

    Ok(())
}

/// Wait for TX DMA_DONE on a specific descriptor.
fn wait_tx_done(
    dev: &Mt76Device, ring: &TxRing, idx: u32,
    timeout_ms: u32, cmd_name: &'static str,
) -> Result<(), McuError> {
    let desc = ring.desc(idx);

    for _ in 0..timeout_ms {
        let ctrl = unsafe { core::ptr::read_volatile(&(*desc).ctrl) };
        if ctrl & MT_DMA_CTL_DMA_DONE != 0 {
            return Ok(());
        }
        userlib::delay_ms(1);
    }

    uerror!("mcu", "tx_timeout"; cmd = cmd_name);
    ulog::flush();
    Err(McuError::TxTimeout { cmd: cmd_name })
}

/// Wait for MCU response on the RX queue associated with the TX ring.
fn wait_rx_response(
    dev: &Mt76Device, ring: &TxRing,
    pre_dma_idx: u32, expected_seq: u8,
    timeout_ms: u32, mut irq: Option<&mut FwIrq>,
    cmd_name: &'static str,
) -> Result<(), McuError> {
    let rx_ndesc = if ring.rx_regs == MCU_WA_RX_REGS {
        MT7996_RX_MCU_RING_SIZE_WA
    } else {
        MT7996_RX_MCU_RING_SIZE
    };

    let mut check_pos = pre_dma_idx;
    let iterations = if irq.is_some() { (timeout_ms + 19) / 20 } else { timeout_ms };

    for _ in 0..iterations {
        loop {
            let dma_idx = dev.rr(ring.rx_regs + MT_QUEUE_DMA_IDX);
            if dma_idx == check_pos { break; }

            if ring.rx_buf_virt != 0 {
                let buf_ptr = (ring.rx_buf_virt + check_pos as u64 * ring.rx_buf_size as u64) as *const u8;
                let rxd_seq = unsafe { core::ptr::read_volatile(buf_ptr.add(37)) };

                if rxd_seq != expected_seq {
                    check_pos = (check_pos + 1) % rx_ndesc;
                    continue;
                }

                // Check status for UNI event responses (eid=1)
                let rxd_eid = unsafe { core::ptr::read_volatile(buf_ptr.add(36)) };
                if rxd_eid == 1 {
                    let resp_cid = unsafe { core::ptr::read_volatile(buf_ptr.add(44)) };
                    let status = unsafe { u32::from_le(core::ptr::read_volatile(buf_ptr.add(48) as *const u32)) };
                    if status != 0 {
                        uerror!("mcu", "cmd_rejected"; cmd = cmd_name, cid = resp_cid,
                            status = status, seq = expected_seq as u32);
                        return Err(McuError::Rejected { cmd: cmd_name, status });
                    }
                }
            }
            return Ok(());
        }

        if let Some(ref mut fw_irq) = irq {
            let _ = fw_irq.wait();
            fw_irq.ack();
            userlib::delay_ms(1);
        } else {
            userlib::delay_ms(1);
        }
    }

    uerror!("mcu", "rx_timeout"; cmd = cmd_name, seq = expected_seq);
    Err(McuError::RxTimeout { cmd: cmd_name, seq: expected_seq })
}

/// Get the RX ring size for a given RX regs base address.
fn rx_ring_size(rx_regs: u32) -> u32 {
    if rx_regs == MCU_WA_RX_REGS {
        MT7996_RX_MCU_RING_SIZE_WA
    } else {
        MT7996_RX_MCU_RING_SIZE
    }
}

/// Clear pending interrupts on both HIFs.
fn clear_pending_interrupts(dev: &Mt76Device) {
    let mask1 = dev.rr(MT_INT_MASK_CSR);
    let mask2 = dev.rr(MT_INT1_MASK_CSR);
    dev.wr(MT_INT_MASK_CSR, 0);
    dev.wr(MT_INT1_MASK_CSR, 0);

    let src1 = dev.rr(MT_INT_SOURCE_CSR);
    dev.wr(MT_INT_SOURCE_CSR, src1);
    let src2 = dev.rr(MT_INT1_SOURCE_CSR);
    dev.wr(MT_INT1_SOURCE_CSR, src2);

    dev.wr(MT_INT_MASK_CSR, mask1);
    dev.wr(MT_INT1_MASK_CSR, mask2);
}

// ============================================================================
// Firmware loading MCU commands
// ============================================================================

/// Request patch semaphore.
pub fn patch_sem_ctrl(
    dev: &Mt76Device, ring: &mut TxRing,
    get: bool, seq: u8, irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let op: u32 = if get { 1 } else { 0 };
    let data = op.to_le_bytes();

    let rx_snap = dev.rr(MCU_WM_RX_REGS + MT_QUEUE_DMA_IDX);
    send_cmd(dev, ring, mcu_cmd::PATCH_SEM_CTRL, &data, seq)?;

    let prev = ring.last_submitted();
    wait_tx_done(dev, ring, prev, 1000, "patch_sem")?;
    wait_rx_wm(dev, rx_snap, 2000, irq, "patch_sem")
}

/// Init download — set target address, length, and mode.
pub fn init_download(
    dev: &Mt76Device, ring: &mut TxRing,
    addr: u32, len: u32, mode: u32,
    seq: u8, irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let cmd = if addr == 0x900000 {
        mcu_cmd::PATCH_START_REQ
    } else {
        mcu_cmd::TARGET_ADDRESS_LEN_REQ
    };

    let mut data = [0u8; 12];
    data[0..4].copy_from_slice(&addr.to_le_bytes());
    data[4..8].copy_from_slice(&len.to_le_bytes());
    data[8..12].copy_from_slice(&(mode | dl_mode::NEED_RSP).to_le_bytes());

    let rx_snap = dev.rr(MCU_WM_RX_REGS + MT_QUEUE_DMA_IDX);
    send_cmd(dev, ring, cmd, &data, seq)?;

    let prev = ring.last_submitted();
    wait_tx_done(dev, ring, prev, 1000, "init_dl")?;
    wait_rx_wm(dev, rx_snap, 2000, irq, "init_dl")
}

/// Send firmware chunk — raw data, NO McuTxd header.
/// Source: mt76_mcu.c __mt76_mcu_send_firmware() — fire-and-forget.
///
/// Unlike MCU commands, firmware scatter chunks are sent as raw bytes
/// on the FWDL ring. The MCU knows how much data to expect from the
/// preceding init_download call.
pub fn send_fw_chunk(
    dev: &Mt76Device, ring: &mut TxRing,
    chunk: &[u8], _seq: u8, _irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let next = (ring.cpu_idx + 1) % ring.ndesc;

    // Ring full check — wait for DMA to consume at least one entry
    if next == dev.rr(ring.regs_base + MT_QUEUE_DMA_IDX) {
        for _ in 0..100u32 {
            if next != dev.rr(ring.regs_base + MT_QUEUE_DMA_IDX) { break; }
            userlib::delay_ms(1);
        }
    }

    ring.submit_raw(dev, chunk);
    Ok(())
}

/// Start firmware execution.
/// For patches: sends PATCH_FINISH_REQ with 4 zero bytes.
/// For RAM images: sends FW_START_REQ with option + override_addr.
pub fn start_firmware(
    dev: &Mt76Device, ring: &mut TxRing,
    is_patch: bool, option: u32, override_addr: u32,
    seq: u8, irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let rx_snap = dev.rr(MCU_WM_RX_REGS + MT_QUEUE_DMA_IDX);

    if is_patch {
        // Patch: PATCH_FINISH_REQ with 4 zero bytes
        let data = [0u8; 4];
        send_cmd(dev, ring, mcu_cmd::PATCH_FINISH_REQ, &data, seq)?;
    } else {
        // RAM: FW_START_REQ with option + override_addr
        let mut data = [0u8; 8];
        data[0..4].copy_from_slice(&option.to_le_bytes());
        data[4..8].copy_from_slice(&override_addr.to_le_bytes());
        send_cmd(dev, ring, mcu_cmd::FW_START_REQ, &data, seq)?;
    }

    let prev = ring.last_submitted();
    wait_tx_done(dev, ring, prev, 1000, "fw_start")?;
    wait_rx_wm(dev, rx_snap, 5000, irq, "fw_start")
}

/// Wait on MCU_WM RX queue (simple DMA_IDX comparison).
fn wait_rx_wm(
    dev: &Mt76Device, pre_dma_idx: u32,
    timeout_ms: u32, mut irq: Option<&mut FwIrq>,
    cmd_name: &'static str,
) -> Result<(), McuError> {
    // Quick check
    if dev.rr(MCU_WM_RX_REGS + MT_QUEUE_DMA_IDX) != pre_dma_idx {
        return Ok(());
    }

    if let Some(ref mut fw_irq) = irq {
        let iterations = (timeout_ms + 19) / 20;
        for _ in 0..iterations {
            let _ = fw_irq.wait();
            let src = dev.rr(MT_INT_SOURCE_CSR);
            if src != 0 { dev.wr(MT_INT_SOURCE_CSR, src); }
            fw_irq.ack();
            if dev.rr(MCU_WM_RX_REGS + MT_QUEUE_DMA_IDX) != pre_dma_idx {
                return Ok(());
            }
        }
    } else {
        for _ in 0..timeout_ms {
            if dev.rr(MCU_WM_RX_REGS + MT_QUEUE_DMA_IDX) != pre_dma_idx {
                return Ok(());
            }
            userlib::delay_ms(1);
        }
    }

    let final_dma = dev.rr(MCU_WM_RX_REGS + MT_QUEUE_DMA_IDX);
    uerror!("mcu", "rx_wm_timeout"; cmd = cmd_name, pre = pre_dma_idx, post = final_dma);
    ulog::flush();
    Err(McuError::RxTimeout { cmd: cmd_name, seq: 0 })
}

// ============================================================================
// Post-firmware MCU commands
// ============================================================================

/// Compute download mode from region's feature_set.
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
// Post-firmware MCU commands — init, EEPROM, radio, interface, beacon, data
// ============================================================================

/// Find the byte offset of an IE (by tag) within a raw 802.11 beacon frame.
/// Searches the IE area after the fixed beacon fields.
pub fn find_ie_offset(frame: &[u8], ie_tag: u8) -> Option<usize> {
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

// --- MCU subsystem init ---

/// Enable firmware logging (WM or WA).
/// Linux: mt7996/mcu.c:3238 mt7996_mcu_fw_log_2_host()
pub fn fw_log_2_host(
    dev: &Mt76Device, ring: &mut TxRing,
    fw_type: u8, ctrl: u8, seq: u8, irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let mut data = [0u8; 12];
    write_tlv(&mut data, 4, UNI_WSYS_CONFIG_FW_LOG_CTRL, 8);
    data[8] = ctrl;

    let cmd = if fw_type == 1 {
        CMD_FIELD_UNI | (MCU_UNI_CMD_WSYS_CONFIG as u32) | CMD_FIELD_WA
    } else {
        CMD_FIELD_UNI | (MCU_UNI_CMD_WSYS_CONFIG as u32) | CMD_FIELD_WM
    };
    send_uni_cmd(dev, ring, cmd, &data, true, seq, irq)
}

/// Enable MWDS (legacy WA EXT command, not UNI).
/// Linux: mt7996/mcu.c:3258 mt7996_mcu_set_mwds()
pub fn set_mwds(
    dev: &Mt76Device, ring: &mut TxRing, enabled: bool, seq: u8,
) -> Result<(), McuError> {
    let mut req = [0u8; 4];
    req[0] = if enabled { 1 } else { 0 };

    send_cmd_ext(dev, ring, mcu_cmd::EXT_CID, MCU_EXT_CMD_MWDS_SUPPORT, S2D_H2C, &req, seq)?;
    let prev = ring.last_submitted();
    wait_tx_done(dev, ring, prev, 2000, "mwds")?;
    userlib::delay_ms(10);
    Ok(())
}

/// Initialize RX airtime for all 3 bands.
/// Linux: mt7996/mcu.c:3288 mt7996_mcu_init_rx_airtime()
pub fn init_rx_airtime(
    dev: &Mt76Device, ring: &mut TxRing, seq: u8, irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let mut data = [0u8; 52]; // uni_header(4) + 6 TLVs × 8 bytes
    let mut offset = 4usize;

    for band in 0u8..3 {
        // CLR_EN
        data[offset..offset + 2].copy_from_slice(&UNI_VOW_RX_AT_AIRTIME_CLR_EN.to_le_bytes());
        data[offset + 2..offset + 4].copy_from_slice(&8u16.to_le_bytes());
        data[offset + 4] = 1;
        data[offset + 5] = band;
        offset += 8;
        // EN
        data[offset..offset + 2].copy_from_slice(&UNI_VOW_RX_AT_AIRTIME_EN.to_le_bytes());
        data[offset + 2..offset + 4].copy_from_slice(&8u16.to_le_bytes());
        data[offset + 4] = 1;
        data[offset + 5] = band;
        offset += 8;
    }

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_VOW as u32) | CMD_FIELD_WM;
    send_uni_cmd(dev, ring, cmd, &data, true, seq, irq)
}

/// Send WA parameter command (legacy MCU, not UNI).
/// Linux: mt7996/mcu.c:365 mt7996_mcu_wa_cmd()
pub fn wa_cmd(
    dev: &Mt76Device, ring: &mut TxRing,
    a1: u32, a2: u32, a3: u32, seq: u8,
) -> Result<(), McuError> {
    let mut args = [0u8; 12];
    args[0..4].copy_from_slice(&a1.to_le_bytes());
    args[4..8].copy_from_slice(&a2.to_le_bytes());
    args[8..12].copy_from_slice(&a3.to_le_bytes());

    send_cmd_ext(dev, ring, mcu_cmd::WA_PARAM, 1, S2D_H2C, &args, seq)?;
    let prev = ring.last_submitted();
    wait_tx_done(dev, ring, prev, 2000, "wa_cmd")?;
    userlib::delay_ms(10);
    Ok(())
}

/// Set RRO module parameters.
/// Linux: mt7996/mcu.c:4688-4739 mt7996_mcu_set_rro()
pub fn set_rro(
    dev: &Mt76Device, ring: &mut TxRing,
    tag: u16, val: u16, seq: u8, irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let mut data = [0u8; 16];
    write_tlv(&mut data, 4, tag, 12);

    match tag {
        UNI_RRO_SET_PLATFORM_TYPE | UNI_RRO_SET_BYPASS_MODE | UNI_RRO_SET_TXFREE_PATH => {
            data[8] = val as u8;
        }
        UNI_RRO_SET_FLUSH_TIMEOUT => {
            data[8..10].copy_from_slice(&val.to_le_bytes());
            data[10..12].copy_from_slice(&(2 * val).to_le_bytes());
        }
        _ => return Err(McuError::Rejected { cmd: "set_rro", status: tag as u32 }),
    }

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_RRO as u32) | CMD_FIELD_WM;
    send_uni_cmd(dev, ring, cmd, &data, true, seq, irq)
}

// --- EEPROM ---

/// Query free eFuse blocks. >=59 means eFuse empty → need flash upload.
/// Linux: mt7996/mcu.c:3872-3900 mt7996_mcu_get_eeprom_free_block()
pub fn get_eeprom_free_block(
    dev: &Mt76Device, ring: &mut TxRing, seq: u8, irq: Option<&mut FwIrq>,
) -> Result<u8, McuError> {
    let mut data = [0u8; 12];
    write_tlv(&mut data, 4, UNI_EFUSE_FREE_BLOCK, 8);
    data[9] = 2; // version

    let rx_snap = dev.rr(ring.rx_regs + MT_QUEUE_DMA_IDX);

    let cmd = CMD_FIELD_UNI | CMD_FIELD_QUERY | (MCU_UNI_CMD_EFUSE_CTRL as u32) | CMD_FIELD_WM;
    send_uni_cmd(dev, ring, cmd, &data, false, seq, None)?;
    wait_rx_response(dev, ring, rx_snap, seq, 5000, irq, "efuse_free")?;

    if ring.rx_buf_virt != 0 {
        let rx_ndesc = rx_ring_size(ring.rx_regs);
        let dma_idx = dev.rr(ring.rx_regs + MT_QUEUE_DMA_IDX);
        let pos = if dma_idx == 0 { rx_ndesc - 1 } else { dma_idx - 1 };
        let buf_ptr = (ring.rx_buf_virt + pos as u64 * ring.rx_buf_size as u64) as *const u8;
        let free_blocks = unsafe { core::ptr::read_volatile(buf_ptr.add(52)) };
        Ok(free_blocks)
    } else {
        Ok(59) // no RX buffer — assume empty
    }
}

/// Tell MCU about EEPROM mode (efuse).
/// Linux: mt7996/mcu.c:3810-3824 mt7996_mcu_set_eeprom()
pub fn set_eeprom(
    dev: &Mt76Device, ring: &mut TxRing, seq: u8, irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let mut data = [0u8; 12];
    write_tlv(&mut data, 4, UNI_EFUSE_BUFFER_MODE, 8);
    data[8] = EE_MODE_EFUSE;
    data[9] = EE_FORMAT_WHOLE;

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_EFUSE_CTRL as u32) | CMD_FIELD_WM;
    send_uni_cmd(dev, ring, cmd, &data, true, seq, irq)
}

/// Read a 16-byte EEPROM block from eFuse.
/// Linux: mt7996/mcu.c:3826-3870 mt7996_mcu_get_eeprom()
pub fn get_eeprom(
    dev: &Mt76Device, ring: &mut TxRing,
    offset: u32, seq: u8, irq: Option<&mut FwIrq>,
) -> Result<[u8; 16], McuError> {
    let mut data = [0u8; 32];
    write_tlv(&mut data, 4, UNI_EFUSE_ACCESS, 28);
    let aligned = offset & !0xF;
    data[8..12].copy_from_slice(&aligned.to_le_bytes());

    let rx_snap = dev.rr(ring.rx_regs + MT_QUEUE_DMA_IDX);

    let cmd = CMD_FIELD_UNI | CMD_FIELD_QUERY | (MCU_UNI_CMD_EFUSE_CTRL as u32) | CMD_FIELD_WM;
    send_uni_cmd(dev, ring, cmd, &data, false, seq, None)?;
    wait_rx_response(dev, ring, rx_snap, seq, 5000, irq, "get_eeprom")?;

    if ring.rx_buf_virt == 0 {
        return Err(McuError::Device(DeviceError::PollTimeout { reg: 0, mask: 0, expected: 0, got: 0 }));
    }

    let rx_ndesc = rx_ring_size(ring.rx_regs);
    let dma_idx = dev.rr(ring.rx_regs + MT_QUEUE_DMA_IDX);
    let pos = if dma_idx == 0 { rx_ndesc - 1 } else { dma_idx - 1 };
    let buf_ptr = (ring.rx_buf_virt + pos as u64 * ring.rx_buf_size as u64) as *const u8;

    // valid at RXD(44)+16=60
    let valid = unsafe { u32::from_le(core::ptr::read_volatile(buf_ptr.add(60) as *const u32)) };
    if valid == 0 {
        return Err(McuError::Rejected { cmd: "get_eeprom", status: 0 });
    }

    let mut result = [0u8; 16];
    for i in 0..16 {
        result[i] = unsafe { core::ptr::read_volatile(buf_ptr.add(44 + 48 + i)) };
    }
    Ok(result)
}

/// Upload EEPROM data to firmware in flash/buffer mode (1024-byte pages).
/// Linux: mt7996/mcu.c:3765-3808 mt7996_mcu_set_eeprom_flash()
pub fn set_eeprom_flash(
    dev: &Mt76Device, ring: &mut TxRing,
    eeprom: &[u8], seq: &mut u8, mut irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let total_pages = (eeprom.len() + PER_PAGE_SIZE - 1) / PER_PAGE_SIZE;
    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_EFUSE_CTRL as u32) | CMD_FIELD_WM;

    for i in 0..total_pages {
        let offset = i * PER_PAGE_SIZE;
        let eep_len = if i == total_pages - 1 && (eeprom.len() % PER_PAGE_SIZE) != 0 {
            eeprom.len() % PER_PAGE_SIZE
        } else {
            PER_PAGE_SIZE
        };

        let msg_len = 4 + 8 + eep_len;
        let format = (((total_pages - 1) as u8) << 5) | ((i as u8) << 2) | EE_FORMAT_WHOLE;

        let mut data = [0u8; 4 + 8 + PER_PAGE_SIZE];
        let data = &mut data[..msg_len];

        write_tlv(data, 4, UNI_EFUSE_BUFFER_MODE, (msg_len - 4) as u16);
        data[8] = EE_MODE_BUFFER;
        data[9] = format;
        data[10..12].copy_from_slice(&(eep_len as u16).to_le_bytes());
        data[12..12 + eep_len].copy_from_slice(&eeprom[offset..offset + eep_len]);

        send_uni_cmd(dev, ring, cmd, data, true, *seq, irq.as_deref_mut())?;
        *seq = seq.wrapping_add(1);
    }
    Ok(())
}

/// Read an RF register via MCU.
/// Linux: mt7996/mcu.c:4633-4670 mt7996_mcu_rf_regval()
pub fn rf_regval(
    dev: &Mt76Device, ring: &mut TxRing,
    regidx: u32, seq: u8, irq: Option<&mut FwIrq>,
) -> Result<u32, McuError> {
    let mut data = [0u8; 20];
    write_tlv(&mut data, 4, UNI_CMD_ACCESS_RF_REG_BASIC, 16);
    let idx = ((regidx >> 24) & 0xFF) as u16;
    data[8..10].copy_from_slice(&idx.to_le_bytes());
    let ofs = regidx & 0x00FFFFFF;
    data[12..16].copy_from_slice(&ofs.to_le_bytes());

    let rx_snap = dev.rr(ring.rx_regs + MT_QUEUE_DMA_IDX);

    let cmd = CMD_FIELD_UNI | CMD_FIELD_QUERY | (MCU_UNI_CMD_REG_ACCESS as u32) | CMD_FIELD_WM;
    send_uni_cmd(dev, ring, cmd, &data, false, seq, None)?;
    wait_rx_response(dev, ring, rx_snap, seq, 5000, irq, "rf_regval")?;

    if ring.rx_buf_virt == 0 {
        return Err(McuError::Device(DeviceError::PollTimeout { reg: 0, mask: 0, expected: 0, got: 0 }));
    }

    let rx_ndesc = rx_ring_size(ring.rx_regs);
    let dma_idx = dev.rr(ring.rx_regs + MT_QUEUE_DMA_IDX);
    let pos = if dma_idx == 0 { rx_ndesc - 1 } else { dma_idx - 1 };
    let buf_ptr = (ring.rx_buf_virt + pos as u64 * ring.rx_buf_size as u64) as *const u8;
    let val = unsafe { u32::from_le(core::ptr::read_volatile(buf_ptr.add(60) as *const u32)) };
    Ok(val)
}

// --- Radio + thermal ---

/// Enable/disable radio for a band.
/// Linux: mt7996/mcu.c:4539-4558 mt7996_mcu_set_radio_en()
pub fn set_radio_en(
    dev: &Mt76Device, ring: &mut TxRing,
    band: u8, enable: bool, seq: u8, irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let mut data = [0u8; 12];
    data[0] = band;
    write_tlv(&mut data, 4, UNI_BAND_CONFIG_RADIO_ENABLE, 8);
    data[8] = if enable { 1 } else { 0 };

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_BAND_CONFIG as u32) | CMD_FIELD_WM;
    send_uni_cmd(dev, ring, cmd, &data, true, seq, irq)
}

/// Enable/disable thermal protection for a band.
/// Linux: mt7996/mcu.c:4105-4143 mt7996_mcu_set_thermal_protect()
pub fn set_thermal_protect(
    dev: &Mt76Device, ring: &mut TxRing,
    band: u8, enable: bool, seq: u8,
) -> Result<(), McuError> {
    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_THERMAL as u32) | CMD_FIELD_WM;

    // Always send DISABLE first
    {
        let mut data = [0u8; 12];
        write_tlv(&mut data, 4, UNI_CMD_THERMAL_PROTECT_DISABLE, 8);
        data[9] = band;
        data[10] = 1; // protect_type
        data[11] = 1; // trigger_type
        send_uni_cmd(dev, ring, cmd, &data, false, seq, None)?;
    }

    if !enable { return Ok(()); }

    // ENABLE with temperature thresholds
    {
        let mut data = [0u8; 24];
        write_tlv(&mut data, 4, UNI_CMD_THERMAL_PROTECT_ENABLE, 20);
        data[9] = band;
        data[10] = 1; // protect_type
        data[11] = 1; // trigger_type
        data[12..16].copy_from_slice(&MT7996_MAX_TEMP.to_le_bytes());
        data[16..20].copy_from_slice(&MT7996_CRIT_TEMP.to_le_bytes());
        data[20..22].copy_from_slice(&10u16.to_le_bytes()); // sustain_time
        send_uni_cmd(dev, ring, cmd, &data, false, seq, None)?;
    }
    Ok(())
}

/// Set thermal throttling duty cycle for a band.
/// Linux: mt7996/mcu.c:4072-4103 mt7996_mcu_set_thermal_throttling()
pub fn set_thermal_throttling(
    dev: &Mt76Device, ring: &mut TxRing,
    band: u8, state: u8, seq: u8,
) -> Result<(), McuError> {
    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_THERMAL as u32) | CMD_FIELD_WM;
    let mut duty_cycle = state;

    for level in 0u8..4 {
        let mut data = [0u8; 12];
        write_tlv(&mut data, 4, UNI_CMD_THERMAL_PROTECT_DUTY_CONFIG, 8);
        data[9] = band;
        data[10] = level;
        data[11] = duty_cycle;
        send_uni_cmd(dev, ring, cmd, &data, false, seq, None)?;
        duty_cycle /= 2;
    }
    Ok(())
}

/// Set RTS threshold for a band.
/// Linux: mt7996/mcu.c:4517-4537 mt7996_mcu_set_rts_thresh()
pub fn set_rts_thresh(
    dev: &Mt76Device, ring: &mut TxRing,
    band: u8, val: u32, seq: u8, irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let mut data = [0u8; 16];
    data[0] = band;
    write_tlv(&mut data, 4, UNI_BAND_CONFIG_RTS_THRESHOLD, 12);
    data[8..12].copy_from_slice(&val.to_le_bytes());
    data[12..16].copy_from_slice(&2u32.to_le_bytes()); // pkt_thresh

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_BAND_CONFIG as u32) | CMD_FIELD_WM;
    send_uni_cmd(dev, ring, cmd, &data, true, seq, irq)
}

/// Set channel info (switch or RX path).
/// Linux: mt7996/mcu.c:3696-3762 mt7996_mcu_set_chan_info()
pub fn set_chan_info(
    dev: &Mt76Device, ring: &mut TxRing,
    band: u8, tag: u16, channel: u8, bw: u8, channel_band: u8,
    sec_ch_offset: i8, seq: u8, irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let mut data = [0u8; 80];
    write_tlv(&mut data, 4, tag, 76);
    data[8] = channel; // control_ch
    // center_ch: for 40MHz, primary +/- 2
    data[9] = match (bw, sec_ch_offset) {
        (1, 1) => channel.wrapping_add(2),
        (1, -1) => channel.wrapping_sub(2),
        _ => channel,
    };
    data[10] = bw;
    data[11] = 2; // tx_path_num (2T2R)
    data[12] = if tag == UNI_CHANNEL_RX_PATH { 0x3 } else { 2 }; // rx_path
    data[13] = CH_SWITCH_NORMAL;
    data[14] = band;
    data[18] = channel_band;

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_CHANNEL_SWITCH as u32) | CMD_FIELD_WM | CMD_FIELD_WA;
    send_uni_cmd(dev, ring, cmd, &data, true, seq, irq)
}

/// Set TX power SKU rate limits.
/// Linux: mt7996/mcu.c:4798-4857 mt7996_mcu_set_txpower_sku()
pub fn set_txpower_sku(
    dev: &Mt76Device, ring: &mut TxRing,
    band: u8, seq: u8, irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    const HEADER_SIZE: usize = 11;
    const TOTAL_SIZE: usize = HEADER_SIZE + MT7996_SKU_PATH_NUM;
    let mut data = [0u8; TOTAL_SIZE];

    write_tlv(&mut data, 4, UNI_TXPOWER_POWER_LIMIT_TABLE_CTRL, (HEADER_SIZE + MT7996_SKU_PATH_NUM - 4) as u16);
    data[8] = UNI_TXPOWER_POWER_LIMIT_TABLE_CTRL as u8;
    data[10] = band;

    // Fill all rate limits with 127 (max s8) — no host-side limit
    for b in &mut data[HEADER_SIZE..] { *b = 127; }

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_TXPOWER as u32) | CMD_FIELD_WM;
    send_uni_cmd(dev, ring, cmd, &data, true, seq, irq)
}

/// Enable/disable RX header translation.
/// Linux: mt7996/mcu.c:3387-3420 mt7996_mcu_set_hdr_trans()
pub fn set_hdr_trans(
    dev: &Mt76Device, ring: &mut TxRing,
    enable: bool, seq: u8, irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let total = if enable { 28 } else { 20 };
    let mut data = [0u8; 28];
    let mut off = 4usize;

    // TLV1: HDR_TRANS_EN
    off = write_tlv(&mut data, off, UNI_HDR_TRANS_EN, 8);
    data[off] = if enable { 1 } else { 0 };
    off += 4;

    // TLV2: HDR_TRANS_VLAN
    off = write_tlv(&mut data, off, UNI_HDR_TRANS_VLAN, 8);
    off += 4;

    // TLV3: HDR_TRANS_BLACKLIST (only if enable)
    if enable {
        write_tlv(&mut data, off, UNI_HDR_TRANS_BLACKLIST, 8);
        data[off + 5] = 1; // enable
        data[off + 6..off + 8].copy_from_slice(&ETH_P_PAE.to_le_bytes());
    }

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_RX_HDR_TRANS as u32) | CMD_FIELD_WM;
    send_uni_cmd(dev, ring, cmd, &data[..total], true, seq, irq)
}

// --- Interface creation ---

/// Add/remove device info (OMAC).
/// Linux: mt7996/mcu.c:2623-2651 mt7996_mcu_add_dev_info()
pub fn add_dev_info(
    dev: &Mt76Device, ring: &mut TxRing,
    band: u8, omac_idx: u8, mac_addr: &[u8; 6], enable: bool,
    seq: u8, irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let mut data = [0u8; 16];
    data[0] = omac_idx;
    data[1] = band;
    write_tlv(&mut data, 4, DEV_INFO_ACTIVE, 12);
    data[8] = if enable { 1 } else { 0 };
    data[10..16].copy_from_slice(mac_addr);

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_DEV_INFO_UPDATE as u32) | CMD_FIELD_WM | CMD_FIELD_WA;
    send_uni_cmd(dev, ring, cmd, &data, true, seq, irq)
}

/// Add/remove BSS info with all required AP TLVs.
/// Linux: mt7996/mcu.c:1028-1090
/// TLVs: BASIC(32) + SEC(8) + RLM(16) + RA(16) + RATE(24) + TXCMD(8) + IFS_TIME(20) + MLD(16) = 144
pub fn add_bss_info(
    dev: &Mt76Device, ring: &mut TxRing,
    band: u8, omac_idx: u8, hw_bss_idx: u8, mac_addr: &[u8; 6],
    enable: bool, channel: u8, bw: u8, sec_ch_offset: i8,
    cipher: u8, seq: u8, irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let mut data = [0u8; 144];

    // BSS_INFO_BASIC TLV (32 bytes)
    let off = 4;
    write_tlv(&mut data, off, UNI_BSS_INFO_BASIC, 32);
    data[off + 4] = if enable { 1 } else { 0 };
    data[off + 5] = omac_idx;
    data[off + 6] = hw_bss_idx;
    data[off + 7] = band;
    data[off + 8..off + 12].copy_from_slice(&CONNECTION_INFRA_AP.to_le_bytes());
    data[off + 12] = if enable { 0 } else { 1 }; // conn_state
    data[off + 14..off + 20].copy_from_slice(mac_addr);
    let bmc_wlan_idx = MT7996_WTBL_RESERVED - hw_bss_idx as u16;
    data[off + 20..off + 22].copy_from_slice(&bmc_wlan_idx.to_le_bytes());
    data[off + 22..off + 24].copy_from_slice(&100u16.to_le_bytes()); // bcn_interval
    data[off + 24] = 1; // dtim_period
    data[off + 25] = PHY_MODE_B | PHY_MODE_G | PHY_MODE_GN;
    data[off + 26..off + 28].copy_from_slice(&bmc_wlan_idx.to_le_bytes());

    // BSS_INFO_SEC TLV (8 bytes)
    let off = 36;
    write_tlv(&mut data, off, UNI_BSS_INFO_SEC, 8);
    data[off + 6] = cipher;

    // BSS_INFO_RLM TLV (16 bytes)
    let off = 44;
    write_tlv(&mut data, off, UNI_BSS_INFO_RLM, 16);
    data[off + 4] = channel;
    data[off + 5] = if bw >= 1 { (channel as i8 + sec_ch_offset * 2) as u8 } else { channel };
    data[off + 7] = bw;
    data[off + 8] = 2; // tx_streams
    data[off + 9] = 2; // rx_streams
    data[off + 12] = 1; // band = 2GHz

    // BSS_INFO_RA TLV (16 bytes)
    let off = 60;
    write_tlv(&mut data, off, UNI_BSS_INFO_RA, 16);
    data[off + 4] = 1; // short_preamble

    // BSS_INFO_RATE TLV (24 bytes)
    let off = 76;
    write_tlv(&mut data, off, UNI_BSS_INFO_RATE, 24);
    data[off + 12] = 1; // short_preamble
    data[off + 13] = MT7996_BASIC_RATES_TBL;
    data[off + 14] = MT7996_BASIC_RATES_TBL;

    // BSS_INFO_TXCMD TLV (8 bytes)
    let off = 100;
    write_tlv(&mut data, off, UNI_BSS_INFO_TXCMD, 8);
    data[off + 4] = 1; // txcmd_mode

    // BSS_INFO_IFS_TIME TLV (20 bytes)
    let off = 108;
    write_tlv(&mut data, off, UNI_BSS_INFO_IFS_TIME, 20);
    data[off + 4] = 1; data[off + 5] = 1; data[off + 6] = 1; data[off + 7] = 1; // valid flags
    data[off + 8..off + 10].copy_from_slice(&9u16.to_le_bytes());    // slot_time
    data[off + 10..off + 12].copy_from_slice(&10u16.to_le_bytes());  // sifs_time
    data[off + 12..off + 14].copy_from_slice(&2u16.to_le_bytes());   // rifs_time
    data[off + 14..off + 16].copy_from_slice(&78u16.to_le_bytes());  // eifs_time (2GHz)
    data[off + 16] = 1; // eifs_cck_valid
    data[off + 18..off + 20].copy_from_slice(&314u16.to_le_bytes()); // eifs_cck_time

    // BSS_INFO_MLD TLV (16 bytes) — mandatory even for non-MLD
    let off = 128;
    write_tlv(&mut data, off, UNI_BSS_INFO_MLD, 16);
    data[off + 4] = 0xff; // group_mld_id
    data[off + 5] = hw_bss_idx; // own_mld_id
    data[off + 12] = 0xff; // remap_idx

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_BSS_INFO_UPDATE as u32) | CMD_FIELD_WM | CMD_FIELD_WA;
    send_uni_cmd(dev, ring, cmd, &data, true, seq, irq)
}

/// Add BMC STA record.
/// Linux: mt7996/mcu.c:2438-2477, mt76_connac_mcu.c:388-415
/// TLVs: STA_REC_BASIC(20) + STA_REC_HDR_TRANS(8) + STA_REC_TX_PROC(8) = 44 bytes
pub fn add_sta(
    dev: &Mt76Device, ring: &mut TxRing,
    bss_idx: u8, wlan_idx: u16, omac_idx: u8, conn_state: u8,
    mac_addr: &[u8; 6], newly: bool,
    seq: u8, irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let mut data = [0u8; 44];

    // sta_req_hdr (8 bytes)
    write_sta_hdr(&mut data, bss_idx, wlan_idx, 0x0e, 3); // muar_idx 0x0e = wildcard for BMC

    // STA_REC_BASIC (20 bytes)
    let off = 8;
    write_tlv(&mut data, off, STA_REC_BASIC, 20);
    data[off + 4..off + 8].copy_from_slice(&CONNECTION_INFRA_BC.to_le_bytes());
    data[off + 8] = conn_state;
    data[off + 12..off + 18].copy_from_slice(&[0xff; 6]); // broadcast
    let mut extra = EXTRA_INFO_VER;
    if newly { extra |= EXTRA_INFO_NEW; }
    data[off + 18..off + 20].copy_from_slice(&extra.to_le_bytes());

    // STA_REC_HDR_TRANS (8 bytes)
    let off = 28;
    write_tlv(&mut data, off, STA_REC_HDR_TRANS, 8);
    data[off + 4] = 1; // from_ds (AP mode)
    data[off + 6] = 1; // dis_rx_hdr_tran (new WCID)

    // STA_REC_TX_PROC (8 bytes)
    let off = 36;
    write_tlv(&mut data, off, STA_REC_TX_PROC, 8);

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_STA_REC_UPDATE as u32) | CMD_FIELD_WM | CMD_FIELD_WA;
    send_uni_cmd(dev, ring, cmd, &data, true, seq, irq)
}

/// Add client STA record (real STA, not BMC).
/// Linux: mt76_connac_mcu.c:388-415 (link_sta != NULL path)
///
/// For CONN_STATE_CONNECT: sends full TLV set (BASIC + HDR_TRANS + TX_PROC + HDRT [+ HT]).
/// For CONN_STATE_DISCONNECT/PORT_SECURE: sends STA_REC_BASIC only (Linux pattern).
pub fn add_client_sta(
    dev: &Mt76Device, ring: &mut TxRing,
    bss_idx: u8, wlan_idx: u16, omac_idx: u8, conn_state: u8,
    mac_addr: &[u8; 6], aid: u16, newly: bool,
    ht_cap: u16, ht_param: u8, sta_flags: u16,
    seq: u8, irq: Option<&mut FwIrq>, wait: bool,
) -> Result<(), McuError> {
    // DISCONNECT/PORT_SECURE: STA_REC_BASIC only (Linux mt7996_mcu_add_sta, sta_state=NONE path)
    // CONNECT: full TLV set for header translation and HT
    let full_tlvs = conn_state == CONN_STATE_CONNECT;
    let has_ht = full_tlvs && (sta_flags & wifi80211::types::STA_FLAG_HT != 0);

    let (tlv_count, total_size): (u16, usize) = if !full_tlvs {
        (1, 28) // sta_req_hdr(8) + STA_REC_BASIC(20) = 28
    } else if has_ht {
        (5, 64) // + HDR_TRANS(8) + TX_PROC(8) + HDRT(8) + HT(12)
    } else {
        (4, 52) // + HDR_TRANS(8) + TX_PROC(8) + HDRT(8)
    };
    let mut data = [0u8; 64];

    // sta_req_hdr
    write_sta_hdr(&mut data, bss_idx, wlan_idx, omac_idx, tlv_count);

    // STA_REC_BASIC
    let off = 8;
    write_tlv(&mut data, off, STA_REC_BASIC, 20);
    data[off + 4..off + 8].copy_from_slice(&CONNECTION_INFRA_STA.to_le_bytes());
    data[off + 8] = conn_state;
    data[off + 9] = 1; // qos
    data[off + 10..off + 12].copy_from_slice(&aid.to_le_bytes());
    data[off + 12..off + 18].copy_from_slice(mac_addr);
    let mut extra = EXTRA_INFO_VER;
    if newly { extra |= EXTRA_INFO_NEW; }
    data[off + 18..off + 20].copy_from_slice(&extra.to_le_bytes());

    if full_tlvs {
        // STA_REC_HDR_TRANS — initially disabled (Linux: dis_rx_hdr_tran=true)
        // Enabled later via wtbl_update_hdr_trans() after decap offload setup.
        let off = 28;
        write_tlv(&mut data, off, STA_REC_HDR_TRANS, 8);
        data[off + 4] = 1; // from_ds
        data[off + 6] = 1; // dis_rx_hdr_tran (disabled initially)

        // STA_REC_TX_PROC
        let off = 36;
        write_tlv(&mut data, off, STA_REC_TX_PROC, 8);

        // STA_REC_HDRT — connac3 header translation mode
        let off = 44;
        write_tlv(&mut data, off, STA_REC_HDRT, 8);
        data[off + 4] = 1; // hdrt_mode

        // STA_REC_HT (12 bytes, only if HT)
        if has_ht {
            let off = 52;
            write_tlv(&mut data, off, STA_REC_HT, 12);
            data[off + 4..off + 6].copy_from_slice(&ht_cap.to_le_bytes());
            data[off + 8] = ht_param;
        }
    }

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_STA_REC_UPDATE as u32) | CMD_FIELD_WM | CMD_FIELD_WA;
    send_uni_cmd(dev, ring, cmd, &data[..total_size], wait, seq, irq)
}

/// Enable RX header translation for a STA (decap offload).
/// Linux: mt7996/mcu.c:4585-4602 mt7996_mcu_wtbl_update_hdr_trans()
///
/// Sends STA_REC_HDR_TRANS with dis_rx_hdr_tran=0 (enabled).
/// Must be called after add_client_sta (which initially disables it).
pub fn wtbl_update_hdr_trans(
    dev: &Mt76Device, ring: &mut TxRing,
    bss_idx: u8, wlan_idx: u16, omac_idx: u8,
    seq: u8,
) -> Result<(), McuError> {
    let mut data = [0u8; 16];

    // sta_req_hdr (8 bytes)
    write_sta_hdr(&mut data, bss_idx, wlan_idx, omac_idx, 1);

    // STA_REC_HDR_TRANS (8 bytes)
    let off = 8;
    write_tlv(&mut data, off, STA_REC_HDR_TRANS, 8);
    data[off + 4] = 1; // from_ds (AP mode)
    // data[off + 5] = 0; // to_ds
    // data[off + 6] = 0; // dis_rx_hdr_tran = 0 → ENABLED
    // data[off + 7] = 0; // mesh

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_STA_REC_UPDATE as u32) | CMD_FIELD_WM | CMD_FIELD_WA;
    send_uni_cmd(dev, ring, cmd, &data, true, seq, None)
}

/// STA rate adaptation control.
/// Linux: mt7996/mcu.c:2169-2274 mt7996_mcu_sta_rate_ctrl_tlv()
pub fn sta_rate_ctrl(
    dev: &Mt76Device, ring: &mut TxRing,
    bss_idx: u8, wlan_idx: u16, omac_idx: u8,
    channel: u8, bw: u8, ht_cap: u16, ht_param: u8, sta_flags: u16,
    seq: u8,
) -> Result<(), McuError> {
    let has_ht = sta_flags & wifi80211::types::STA_FLAG_HT != 0;
    let is_5g = channel >= 36;
    let mut data = [0u8; 68];

    // sta_req_hdr
    write_sta_hdr(&mut data, bss_idx, wlan_idx, omac_idx, 1);

    // STA_REC_RA (60 bytes)
    let off = 8;
    write_tlv(&mut data, off, STA_REC_RA, 60);
    data[off + 4] = 1; // valid
    data[off + 5] = 1; // auto_rate

    let phy_mode: u8 = if is_5g {
        if has_ht { PHY_MODE_A | PHY_MODE_AN } else { PHY_MODE_A }
    } else {
        if has_ht { PHY_MODE_B | PHY_MODE_G | PHY_MODE_GN } else { PHY_MODE_B | PHY_MODE_G }
    };
    data[off + 6] = phy_mode;
    data[off + 7] = channel;
    data[off + 8] = bw;
    data[off + 9] = if is_5g { 1 } else { 0 }; // disable_cck

    if has_ht { data[off + 12] = 0xFF; } // ht_mcs[0]
    data[off + 16] = 3; // mmps_mode = disabled
    data[off + 18] = ht_param & 0x03; // af = ampdu_factor
    data[off + 20] = if is_5g { 8 } else { 12 }; // rate_len
    data[off + 21] = phy_mode;
    data[off + 22] = if is_5g { 0 } else { 0x0F }; // supp_cck_rate
    data[off + 23] = 0xFF; // supp_ofdm_rate
    if has_ht {
        data[off + 24..off + 28].copy_from_slice(&0x000000FFu32.to_le_bytes());
    }

    let mut sta_cap_val: u32 = STA_CAP_WMM;
    if has_ht {
        sta_cap_val |= STA_CAP_HT;
        if ht_cap & 0x0020 != 0 { sta_cap_val |= STA_CAP_SGI_20; }
        if ht_cap & 0x0040 != 0 { sta_cap_val |= STA_CAP_SGI_40; }
        if ht_cap & 0x0080 != 0 { sta_cap_val |= STA_CAP_TX_STBC; }
        if ht_cap & 0x0300 != 0 { sta_cap_val |= STA_CAP_RX_STBC; }
        if ht_cap & 0x0001 != 0 { sta_cap_val |= STA_CAP_LDPC; }
    }
    data[off + 40..off + 44].copy_from_slice(&sta_cap_val.to_le_bytes());

    data[off + 56] = INIT_RCPI;
    data[off + 57] = INIT_RCPI;
    data[off + 58] = INIT_RCPI;
    data[off + 59] = INIT_RCPI;

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_STA_REC_UPDATE as u32) | CMD_FIELD_WM | CMD_FIELD_WA;
    send_uni_cmd(dev, ring, cmd, &data, false, seq, None)
}

/// BA session start/stop notification.
/// Linux: mt7996/mcu.c:1191-1218 mt7996_mcu_sta_ba()
pub fn sta_ba(
    dev: &Mt76Device, ring: &mut TxRing,
    bss_idx: u8, wlan_idx: u16, omac_idx: u8,
    tid: u8, ssn: u16, win_size: u16, enable: bool,
    seq: u8,
) -> Result<(), McuError> {
    let mut data = [0u8; 24];

    write_sta_hdr(&mut data, bss_idx, wlan_idx, omac_idx, 1);

    let off = 8;
    write_tlv(&mut data, off, STA_REC_BA, 16);
    data[off + 4] = tid;
    data[off + 5] = 2; // ba_type = RECIPIENT
    data[off + 7] = if enable { 1u8 << tid } else { 0 };
    data[off + 8..off + 10].copy_from_slice(&ssn.to_le_bytes());
    data[off + 10..off + 12].copy_from_slice(&win_size.to_le_bytes());

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_STA_REC_UPDATE as u32) | CMD_FIELD_WM | CMD_FIELD_WA;
    send_uni_cmd(dev, ring, cmd, &data, false, seq, None)
}

/// Install a cipher key for a STA via STA_REC_KEY_V2.
/// Linux: mt7996/mcu.c:2532-2600 mt7996_mcu_sta_key_tlv()
pub fn install_key(
    dev: &Mt76Device, ring: &mut TxRing,
    bss_idx: u8, wlan_idx: u16, omac_idx: u8,
    key_id: u8, key: &[u8; 16],
    seq: u8, irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let mut data = [0u8; 112]; // sta_req_hdr(8) + sta_rec_sec_uni(104)

    write_sta_hdr(&mut data, bss_idx, wlan_idx, omac_idx, 1);

    let off = 8;
    write_tlv(&mut data, off, STA_REC_KEY_V2, 104);
    data[off + 4] = 0; // add = SET_KEY (Linux enum: SET_KEY=0, DISABLE_KEY=1)
    data[off + 5] = 1; // n_cipher

    let sk = off + 8;
    data[sk..sk + 2].copy_from_slice(&wlan_idx.to_le_bytes());
    data[sk + 3] = MCU_CIPHER_AES_CCMP;
    data[sk + 4] = 48; // cipher_len
    data[sk + 5] = key_id;
    data[sk + 6] = 16; // key_len
    data[sk + 8..sk + 24].copy_from_slice(key);

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_STA_REC_UPDATE as u32) | CMD_FIELD_WM | CMD_FIELD_WA;
    send_uni_cmd(dev, ring, cmd, &data, true, seq, irq)
}

/// Update BSS cipher mode.
/// Linux: mt7996/mcu.c:953-963 mt7996_mcu_bss_sec_tlv()
pub fn update_bss_sec(
    dev: &Mt76Device, ring: &mut TxRing,
    hw_bss_idx: u8, cipher: u8,
    seq: u8, irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let mut data = [0u8; 12];
    data[0] = hw_bss_idx;

    let off = 4;
    write_tlv(&mut data, off, UNI_BSS_INFO_SEC, 8);
    data[off + 6] = cipher;

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_BSS_INFO_UPDATE as u32) | CMD_FIELD_WM | CMD_FIELD_WA;
    send_uni_cmd(dev, ring, cmd, &data, true, seq, irq)
}

/// Assign WCID to BSS scheduling group (VOW DRR control).
/// Linux: mt7996/mcu.c:2343-2367 mt7996_mcu_add_group()
pub fn add_group(
    dev: &Mt76Device, ring: &mut TxRing,
    bss_idx: u8, wlan_idx: u16,
    seq: u8, irq: Option<&mut FwIrq>, wait: bool,
) -> Result<(), McuError> {
    let mut data = [0u8; 24];

    let off = 4;
    write_tlv(&mut data, off, UNI_VOW_DRR_CTRL, 20);
    data[off + 4..off + 6].copy_from_slice(&wlan_idx.to_le_bytes());
    data[off + 8..off + 12].copy_from_slice(&1u32.to_le_bytes()); // action = MT_STA_BSS_GROUP
    data[off + 12..off + 16].copy_from_slice(&((bss_idx as u32) % 16).to_le_bytes());

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_VOW as u32) | CMD_FIELD_WM;
    send_uni_cmd(dev, ring, cmd, &data, wait, seq, irq)
}

// --- Beamforming + beacon + rates ---

/// Initialize transmit beamforming subsystem (3 MCU commands).
/// Linux: mt7996/init.c:640-658 mt7996_txbf_init()
pub fn txbf_init(
    dev: &Mt76Device, ring: &mut TxRing,
    seq: &mut u8, mut irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_BF as u32) | CMD_FIELD_WM;

    // BF_MOD_EN_CTRL (tag=20)
    {
        let mut data = [0u8; 20];
        write_tlv(&mut data, 4, 20, 16);
        data[8] = 3;    // bf_num
        data[9] = 0x07; // bf_bitmap
        send_uni_cmd(dev, ring, cmd, &data, true, *seq, irq.as_deref_mut())?;
        *seq = seq.wrapping_add(1);
    }

    // BF_SOUNDING_ON (tag=1)
    {
        let mut data = [0u8; 24];
        write_tlv(&mut data, 4, 1, 20);
        data[8] = 4; // snd_mode = BF_PROCESSING
        send_uni_cmd(dev, ring, cmd, &data, true, *seq, irq.as_deref_mut())?;
        *seq = seq.wrapping_add(1);
    }

    // BF_HW_EN_UPDATE (tag=17)
    {
        let mut data = [0u8; 12];
        write_tlv(&mut data, 4, 17, 8);
        data[8] = 1; // ebf = true
        send_uni_cmd(dev, ring, cmd, &data, true, *seq, irq)?;
        *seq = seq.wrapping_add(1);
    }

    Ok(())
}

/// Send EDCA/WMM queue parameters for AP mode.
/// Linux: mt7996/mcu.c:3423-3481 mt7996_mcu_set_tx()
pub fn set_edca(
    dev: &Mt76Device, ring: &mut TxRing,
    bss_idx: u8, seq: u8, irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let mut data = [0u8; 52]; // hdr(4) + 4 × edca_tlv(12)
    data[0] = bss_idx;

    // (queue, cw_min, cw_max, txop, aifs)
    const EDCA: [(u8, u8, u8, u16, u8); 4] = [
        (0, 2, 3, 47, 2),   // AC_VO
        (1, 3, 4, 94, 2),   // AC_VI
        (2, 4, 10, 0, 3),   // AC_BE
        (3, 4, 10, 0, 7),   // AC_BK
    ];

    for (i, &(queue, cw_min, cw_max, txop, aifs)) in EDCA.iter().enumerate() {
        let off = 4 + i * 12;
        write_tlv(&mut data, off, 0, 12);
        data[off + 4] = queue;
        data[off + 5] = 0x0F; // WMM_PARAM_SET
        data[off + 6] = cw_min;
        data[off + 7] = cw_max;
        data[off + 8..off + 10].copy_from_slice(&txop.to_le_bytes());
        data[off + 10] = aifs;
    }

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_EDCA_UPDATE as u32) | CMD_FIELD_WM;
    send_uni_cmd(dev, ring, cmd, &data, true, seq, irq)
}

/// Upload beacon template to firmware.
/// Linux: mt7996/mcu.c:2766 mt7996_mcu_add_beacon()
pub fn set_beacon(
    dev: &Mt76Device, ring: &mut TxRing,
    band: u8, omac_idx: u8, beacon_frame: &[u8], enable: bool,
    seq: u8, irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    const MAX_BEACON: usize = 256;
    let bcn_len = beacon_frame.len().min(MAX_BEACON);

    let tlv_body_len = 14 + MT_TXD_SIZE + bcn_len;
    let tlv_aligned = (tlv_body_len + 3) & !3;
    let data_len = 4 + tlv_aligned;

    let mut data = [0u8; 4 + 14 + MT_TXD_SIZE + MAX_BEACON + 4]; // 310
    if data_len > data.len() {
        return Err(McuError::RingFull);
    }

    // bcn_content_tlv header (14 bytes)
    let off = 4usize;
    write_tlv(&mut data, off, UNI_BSS_INFO_BCN_CONTENT, tlv_aligned as u16);
    let tim_pos = find_ie_offset(beacon_frame, 5).unwrap_or(59) as u16;
    data[off + 4..off + 6].copy_from_slice(&tim_pos.to_le_bytes());
    data[off + 10] = if enable { 1 } else { 0 };
    data[off + 12..off + 14].copy_from_slice(&((MT_TXD_SIZE + bcn_len) as u16).to_le_bytes());

    // TXD (32 bytes)
    let txd_off = off + 14;
    let txd0 = ((MT_TXD_SIZE + bcn_len) as u32)
        | ((MT_TX_TYPE_FW as u32) << 23)
        | ((MT_LMAC_BCN0 as u32) << 25);
    data[txd_off..txd_off + 4].copy_from_slice(&txd0.to_le_bytes());

    let txd1 = ((MT_HDR_FORMAT_802_11 as u32) << 14)
        | (12u32 << 16)
        | ((omac_idx as u32) << 25)
        | (1u32 << 31);
    data[txd_off + 4..txd_off + 8].copy_from_slice(&txd1.to_le_bytes());

    let txd2 = 8u32; // FRAME_TYPE(mgmt) + SUB_TYPE(beacon=8)
    data[txd_off + 8..txd_off + 12].copy_from_slice(&txd2.to_le_bytes());

    let txd3 = 1u32 | (1u32 << 4) | (0x1Fu32 << 11) | (1u32 << 28);
    data[txd_off + 12..txd_off + 16].copy_from_slice(&txd3.to_le_bytes());

    let txd6 = (1u32 << 2) | (1u32 << 3) | (1u32 << 4)
        | ((MT7996_BEACON_RATES_TBL as u32) << 16)
        | (1u32 << 25) | (1u32 << 28);
    data[txd_off + 24..txd_off + 28].copy_from_slice(&txd6.to_le_bytes());

    // Copy beacon frame
    let bcn_off = txd_off + MT_TXD_SIZE;
    data[bcn_off..bcn_off + bcn_len].copy_from_slice(&beacon_frame[..bcn_len]);
    data[bcn_off + 22] = 0; // seq_ctrl cleared
    data[bcn_off + 23] = 0;

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_BSS_INFO_UPDATE as u32) | CMD_FIELD_WM | CMD_FIELD_WA;
    send_uni_cmd(dev, ring, cmd, &data[..data_len], true, seq, irq)
}

/// Program a fixed rate table entry.
/// Linux: mt7996/mcu.c:4604-4631 mt7996_mcu_set_fixed_rate_table()
pub fn set_fixed_rate_table(
    dev: &Mt76Device, ring: &mut TxRing,
    table_idx: u8, rate_idx: u16, is_beacon: bool, band: u8,
    seq: u8, irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let mut data = [0u8; 20];
    write_tlv(&mut data, 4, 0, 16);
    data[8] = table_idx;
    data[10..12].copy_from_slice(&rate_idx.to_le_bytes());
    if is_beacon {
        data[13] = 24 + band; // spe_idx
    } else {
        data[12] = 1; // spe_idx_sel = BMC_WTBL
    }
    data[14] = 1; // gi
    data[15] = 1; // he_ltf

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_FIXED_RATE_TABLE as u32) | CMD_FIELD_WM;
    send_uni_cmd(dev, ring, cmd, &data, false, seq, irq)
}

/// Background chain control — start/stop offchannel scan.
/// Linux: mt7996/mcu.c:3604-3658 mt7996_mcu_background_chain_ctrl()
pub fn background_chain_ctrl(
    dev: &Mt76Device, ring: &mut TxRing,
    band: u8, channel: u8, bw: u8, ch_band: u8, scan_mode: u8,
    seq: u8,
) -> Result<(), McuError> {
    let mut data = [0u8; 24];
    write_tlv(&mut data, 4, 0, 20);
    data[20] = 2; // monitor_scan_type

    if scan_mode == 1 {
        data[8] = channel;
        data[9] = channel;
        data[10] = bw;
        data[13] = channel;
        data[14] = channel;
        data[15] = bw;
        data[18] = 1;
        data[19] = band;
    } else {
        data[8] = channel;
        data[9] = channel;
        data[10] = bw;
        data[11] = 2; // tx_stream
        data[12] = 3; // rx_stream
    }
    data[21] = ch_band;

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_OFFCH_SCAN_CTRL as u32) | CMD_FIELD_WM;
    send_uni_cmd(dev, ring, cmd, &data, false, seq, None)
}

// --- Periodic / fire-and-forget ---

/// Query all STA info (fire-and-forget heartbeat).
/// Linux: mt7996/mcu.c:4741-4756 mt7996_mcu_get_all_sta_info()
pub fn get_all_sta_info(
    dev: &Mt76Device, ring: &mut TxRing, tag: u16, seq: u8,
) -> Result<(), McuError> {
    let mut data = [0u8; 8];
    write_tlv(&mut data, 4, tag, 4);

    let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_ALL_STA_INFO as u32) | CMD_FIELD_WM;
    send_uni_cmd(dev, ring, cmd, &data, false, seq, None)
}

/// Read and clear channel MIB counters (fire-and-forget).
/// Linux: mt7996/mcu.c:3948-4025 mt7996_mcu_get_chan_mib_info()
///
/// Without periodic clearing, firmware CCA counters accumulate indefinitely,
/// causing the MAC to stop TX after ~42 seconds.
pub fn get_chan_mib_info(
    dev: &Mt76Device, ring: &mut TxRing, band: u8, seq: u8,
) -> Result<(), McuError> {
    let mut data = [0u8; 36];
    data[0] = band;

    let offsets: [u32; 4] = [
        UNI_MIB_TX_TIME, UNI_MIB_RX_TIME, UNI_MIB_OBSS_AIRTIME, UNI_MIB_NON_WIFI_TIME,
    ];

    for i in 0..4 {
        let base = 4 + i * 8;
        write_tlv(&mut data, base, UNI_CMD_MIB_DATA, 8);
        data[base + 4..base + 8].copy_from_slice(&offsets[i].to_le_bytes());
    }

    let cmd = CMD_FIELD_UNI | CMD_FIELD_QUERY | (MCU_UNI_CMD_GET_MIB_INFO as u32) | CMD_FIELD_WM;
    send_uni_cmd(dev, ring, cmd, &data, false, seq, None)
}
