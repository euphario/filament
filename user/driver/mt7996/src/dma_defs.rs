//! DMA Descriptor Definitions
//!
//! Direct translation of mt76/dma.h from Linux.
//! IMPORTANT: Do not modify - keep 1:1 with Linux.
//!
//! Source: https://github.com/openwrt/mt76/blob/master/dma.h

#![allow(dead_code)]

use super::regs::{bit, genmask};

// ============================================================================
// Ring Size
// ============================================================================

/// Ring register block size (4 registers × 4 bytes)
/// From dma.h: #define MT_RING_SIZE 0x10
pub const MT_RING_SIZE: u32 = 0x10;

// ============================================================================
// DMA Control Bits (ctrl field in mt76_desc)
// ============================================================================

/// SD_LEN1 - Scatter/gather length 1 (bits 13:0)
/// From dma.h: #define MT_DMA_CTL_SD_LEN1 GENMASK(13, 0)
pub const MT_DMA_CTL_SD_LEN1: u32 = genmask(13, 0);

/// LAST_SEC1 - Last section 1 flag (bit 14)
/// From dma.h: #define MT_DMA_CTL_LAST_SEC1 BIT(14)
pub const MT_DMA_CTL_LAST_SEC1: u32 = bit(14);

/// BURST - Burst mode flag (bit 15)
/// From dma.h: #define MT_DMA_CTL_BURST BIT(15)
pub const MT_DMA_CTL_BURST: u32 = bit(15);

/// SD_LEN0 - Scatter/gather length 0 (bits 29:16)
/// From dma.h: #define MT_DMA_CTL_SD_LEN0 GENMASK(29, 16)
pub const MT_DMA_CTL_SD_LEN0: u32 = genmask(29, 16);

/// LAST_SEC0 - Last section 0 flag (bit 30)
/// From dma.h: #define MT_DMA_CTL_LAST_SEC0 BIT(30)
pub const MT_DMA_CTL_LAST_SEC0: u32 = bit(30);

/// DMA_DONE - DMA transfer complete flag (bit 31)
/// From dma.h: #define MT_DMA_CTL_DMA_DONE BIT(31)
pub const MT_DMA_CTL_DMA_DONE: u32 = bit(31);

/// TO_HOST - Transfer to host flag (bit 8)
/// From dma.h: #define MT_DMA_CTL_TO_HOST BIT(8)
pub const MT_DMA_CTL_TO_HOST: u32 = bit(8);

/// TO_HOST_A - Transfer to host A flag (bit 12)
/// From dma.h: #define MT_DMA_CTL_TO_HOST_A BIT(12)
pub const MT_DMA_CTL_TO_HOST_A: u32 = bit(12);

/// DROP - Drop packet flag (bit 14)
/// From dma.h: #define MT_DMA_CTL_DROP BIT(14)
pub const MT_DMA_CTL_DROP: u32 = bit(14);

/// TOKEN - Token ID (bits 31:16)
/// From dma.h: #define MT_DMA_CTL_TOKEN GENMASK(31, 16)
pub const MT_DMA_CTL_TOKEN: u32 = genmask(31, 16);

/// SDP1_H - Scatter data pointer 1 high bits (bits 19:16)
/// From dma.h: #define MT_DMA_CTL_SDP1_H GENMASK(19, 16)
pub const MT_DMA_CTL_SDP1_H: u32 = genmask(19, 16);

/// SDP0_H - Scatter data pointer 0 high bits (bits 3:0)
/// From dma.h: #define MT_DMA_CTL_SDP0_H GENMASK(3, 0)
pub const MT_DMA_CTL_SDP0_H: u32 = genmask(3, 0);

/// WO_DROP - WED offload drop flag (bit 8)
/// From dma.h: #define MT_DMA_CTL_WO_DROP BIT(8)
pub const MT_DMA_CTL_WO_DROP: u32 = bit(8);

// ============================================================================
// PPE (Packet Processing Engine) Bits
// ============================================================================

/// PPE_CPU_REASON - CPU reason (bits 15:11)
/// From dma.h: #define MT_DMA_PPE_CPU_REASON GENMASK(15, 11)
pub const MT_DMA_PPE_CPU_REASON: u32 = genmask(15, 11);

/// PPE_ENTRY - PPE entry (bits 30:16)
/// From dma.h: #define MT_DMA_PPE_ENTRY GENMASK(30, 16)
pub const MT_DMA_PPE_ENTRY: u32 = genmask(30, 16);

/// DMA_FRAG - DMA fragment flag (bit 9)
/// From dma.h: #define MT_DMA_INFO_DMA_FRAG BIT(9)
pub const MT_DMA_INFO_DMA_FRAG: u32 = bit(9);

/// PPE_VLD - PPE valid flag (bit 31)
/// From dma.h: #define MT_DMA_INFO_PPE_VLD BIT(31)
pub const MT_DMA_INFO_PPE_VLD: u32 = bit(31);

// ============================================================================
// PN Check Bits
// ============================================================================

/// PN_CHK_FAIL - PN check fail flag (bit 13)
/// From dma.h: #define MT_DMA_CTL_PN_CHK_FAIL BIT(13)
pub const MT_DMA_CTL_PN_CHK_FAIL: u32 = bit(13);

/// VER_MASK - Version mask (bit 7)
/// From dma.h: #define MT_DMA_CTL_VER_MASK BIT(7)
pub const MT_DMA_CTL_VER_MASK: u32 = bit(7);

// ============================================================================
// Token/Magic Bits
// ============================================================================

/// SDP0 - Scatter data pointer 0 (bits 15:0)
/// From dma.h: #define MT_DMA_SDP0 GENMASK(15, 0)
pub const MT_DMA_SDP0: u32 = genmask(15, 0);

/// TOKEN_ID - Token ID (bits 31:16)
/// From dma.h: #define MT_DMA_TOKEN_ID GENMASK(31, 16)
pub const MT_DMA_TOKEN_ID: u32 = genmask(31, 16);

/// MAGIC_MASK - Magic counter mask (bits 31:28)
/// From dma.h: #define MT_DMA_MAGIC_MASK GENMASK(31, 28)
/// CRITICAL: This is in the INFO field, bits 31:28, NOT bits 15:12!
pub const MT_DMA_MAGIC_MASK: u32 = genmask(31, 28);

/// RRO_EN - RRO enable flag (bit 13)
/// From dma.h: #define MT_DMA_RRO_EN BIT(13)
pub const MT_DMA_RRO_EN: u32 = bit(13);

/// MAGIC_CNT - Magic counter value (for RRO rings)
/// From dma.h: #define MT_DMA_MAGIC_CNT 16
pub const MT_DMA_MAGIC_CNT: u32 = 16;

// ============================================================================
// WED Indication Bits
// ============================================================================

/// WED_IND_CMD_CNT - WED indication command count
/// From dma.h: #define MT_DMA_WED_IND_CMD_CNT 8
pub const MT_DMA_WED_IND_CMD_CNT: u32 = 8;

/// WED_IND_REASON - WED indication reason (bits 15:12)
/// From dma.h: #define MT_DMA_WED_IND_REASON GENMASK(15, 12)
pub const MT_DMA_WED_IND_REASON: u32 = genmask(15, 12);

// ============================================================================
// Header/Info Lengths
// ============================================================================

/// DMA header length
/// From dma.h: #define MT_DMA_HDR_LEN 4
pub const MT_DMA_HDR_LEN: u32 = 4;

/// RX info length
/// From dma.h: #define MT_RX_INFO_LEN 4
pub const MT_RX_INFO_LEN: u32 = 4;

/// FCE info length
/// From dma.h: #define MT_FCE_INFO_LEN 4
pub const MT_FCE_INFO_LEN: u32 = 4;

/// RX RXWI length
/// From dma.h: #define MT_RX_RXWI_LEN 32
pub const MT_RX_RXWI_LEN: u32 = 32;

// ============================================================================
// RRO RXDMAD Bits
// ============================================================================

/// RRO RXDMAD DATA1 last segment mask (bit 30)
/// From dma.h: #define RRO_RXDMAD_DATA1_LS_MASK BIT(30)
pub const RRO_RXDMAD_DATA1_LS_MASK: u32 = bit(30);

/// RRO RXDMAD DATA1 SDL0 mask (bits 29:16)
/// From dma.h: #define RRO_RXDMAD_DATA1_SDL0_MASK GENMASK(29, 16)
pub const RRO_RXDMAD_DATA1_SDL0_MASK: u32 = genmask(29, 16);

/// RRO RXDMAD DATA2 RX token ID mask (bits 31:16)
/// From dma.h: #define RRO_RXDMAD_DATA2_RX_TOKEN_ID_MASK GENMASK(31, 16)
pub const RRO_RXDMAD_DATA2_RX_TOKEN_ID_MASK: u32 = genmask(31, 16);

/// RRO RXDMAD DATA2 indication reason mask (bits 15:12)
/// From dma.h: #define RRO_RXDMAD_DATA2_IND_REASON_MASK GENMASK(15, 12)
pub const RRO_RXDMAD_DATA2_IND_REASON_MASK: u32 = genmask(15, 12);

/// RRO RXDMAD DATA3 magic counter mask (bits 31:28)
/// From dma.h: #define RRO_RXDMAD_DATA3_MAGIC_CNT_MASK GENMASK(31, 28)
pub const RRO_RXDMAD_DATA3_MAGIC_CNT_MASK: u32 = genmask(31, 28);

// ============================================================================
// Enums
// ============================================================================

/// Queue selection types
/// From dma.h: enum mt76_qsel
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt76Qsel {
    Mgmt = 0,
    Hcca = 1,
    Edca = 2,
    Edca2 = 3,
}

/// MCU event types
/// From dma.h: enum mt76_mcu_evt_type
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt76McuEvtType {
    CmdDone = 0,
    CmdError = 1,
    CmdRetry = 2,
    EventPwrRsp = 3,
    EventWowRsp = 4,
    EventCarrierDetectRsp = 5,
    EventDfsDetectRsp = 6,
}

/// WED indication reason
/// From dma.h: enum mt76_dma_wed_ind_reason
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt76DmaWedIndReason {
    Normal = 0,
    Repeat = 1,
    OldPkt = 2,
}

// ============================================================================
// DMA Descriptor Structure
// ============================================================================

/// DMA descriptor (matches struct mt76_desc in dma.h)
///
/// From dma.h:
/// ```c
/// struct mt76_desc {
///     __le32 buf0;
///     __le32 ctrl;
///     __le32 buf1;
///     __le32 info;
/// } __packed __aligned(4);
/// ```
#[repr(C, align(4))]
#[derive(Debug, Clone, Copy, Default)]
pub struct Mt76Desc {
    /// Buffer 0 address (low 32 bits of DMA address)
    pub buf0: u32,
    /// Control field (length, flags, DMA_DONE)
    pub ctrl: u32,
    /// Buffer 1 address (high 32 bits of DMA address, or second buffer)
    pub buf1: u32,
    /// Info field (token, magic counter for RRO)
    pub info: u32,
}

impl Mt76Desc {
    /// Create a new empty descriptor
    pub const fn new() -> Self {
        Self {
            buf0: 0,
            ctrl: 0,
            buf1: 0,
            info: 0,
        }
    }

    /// Create a TX descriptor for firmware download
    ///
    /// This matches the Linux mt76_dma_add_buf() for TX with a single buffer.
    ///
    /// # Arguments
    /// * `dma_addr` - Physical/DMA address of the buffer
    /// * `len` - Length of the data in bytes
    pub fn new_tx(dma_addr: u64, len: u32) -> Self {
        let buf0 = dma_addr as u32;
        let buf1 = (dma_addr >> 32) as u32;

        // ctrl field layout for TX:
        // bits 29:16 = SD_LEN0 (length)
        // bit 30 = LAST_SEC0 (always set for single buffer)
        // bit 31 = DMA_DONE (set by hardware when complete, start with 0 for TX)
        let ctrl = ((len & 0x3FFF) << 16) | MT_DMA_CTL_LAST_SEC0;

        Self {
            buf0,
            ctrl,
            buf1,
            info: 0,
        }
    }

    /// Get the SD_LEN0 (scatter data length 0) from ctrl field
    pub fn sd_len0(&self) -> u32 {
        (self.ctrl & MT_DMA_CTL_SD_LEN0) >> 16
    }

    /// Check if LAST_SEC0 is set
    pub fn is_last_sec0(&self) -> bool {
        (self.ctrl & MT_DMA_CTL_LAST_SEC0) != 0
    }

    /// Check if DMA_DONE is set
    pub fn is_dma_done(&self) -> bool {
        (self.ctrl & MT_DMA_CTL_DMA_DONE) != 0
    }

    /// Set DMA_DONE flag (used by RX descriptors to indicate ownership)
    pub fn set_dma_done(&mut self) {
        self.ctrl |= MT_DMA_CTL_DMA_DONE;
    }

    /// Clear DMA_DONE flag (used when giving descriptor to hardware)
    pub fn clear_dma_done(&mut self) {
        self.ctrl &= !MT_DMA_CTL_DMA_DONE;
    }

    /// Get magic counter from info field (for RRO rings)
    /// CRITICAL: Magic counter is in bits 31:28, NOT bits 15:12!
    pub fn magic_cnt(&self) -> u32 {
        (self.info & MT_DMA_MAGIC_MASK) >> 28
    }

    /// Set magic counter in info field (for RRO rings)
    pub fn set_magic_cnt(&mut self, cnt: u32) {
        self.info = (self.info & !MT_DMA_MAGIC_MASK) | ((cnt & 0xF) << 28);
    }
}

/// WED RRO descriptor (for reorder buffer)
///
/// From dma.h:
/// ```c
/// struct mt76_wed_rro_desc {
///     __le32 buf0;
///     __le32 buf1;
/// } __packed __aligned(4);
/// ```
#[repr(C, align(4))]
#[derive(Debug, Clone, Copy, Default)]
pub struct Mt76WedRroDesc {
    pub buf0: u32,
    pub buf1: u32,
}

/// RRO RXDMAD C descriptor
///
/// From dma.h:
/// ```c
/// struct mt76_rro_rxdmad_c {
///     __le32 data0;
///     __le32 data1;
///     __le32 data2;
///     __le32 data3;
/// };
/// ```
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Mt76RroRxdmadC {
    pub data0: u32,
    pub data1: u32,
    pub data2: u32,
    pub data3: u32,
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Helper to extract field from value using mask
#[inline]
pub const fn field_get(mask: u32, val: u32) -> u32 {
    let shift = mask.trailing_zeros();
    (val & mask) >> shift
}

/// Helper to prepare value for field using mask
#[inline]
pub const fn field_prep(mask: u32, val: u32) -> u32 {
    let shift = mask.trailing_zeros();
    (val << shift) & mask
}

/// Check if packet should be dropped based on control bits
///
/// Translated from dma.h: mt76_dma_should_drop_buf()
pub fn should_drop_buf(ctrl: u32, buf1: u32, info: u32) -> bool {
    // Drop if TO_HOST_A or DROP flags are set
    if (ctrl & (MT_DMA_CTL_TO_HOST_A | MT_DMA_CTL_DROP)) != 0 {
        return true;
    }

    // If VER_MASK is not set, no further checks
    if (ctrl & MT_DMA_CTL_VER_MASK) == 0 {
        return false;
    }

    // Check WED indication reason
    match field_get(MT_DMA_WED_IND_REASON, buf1) {
        r if r == Mt76DmaWedIndReason::Repeat as u32 => true,
        r if r == Mt76DmaWedIndReason::OldPkt as u32 => (info & MT_DMA_INFO_DMA_FRAG) == 0,
        _ => (ctrl & MT_DMA_CTL_PN_CHK_FAIL) != 0,
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_genmask() {
        assert_eq!(MT_DMA_CTL_SD_LEN1, 0x3FFF);
        assert_eq!(MT_DMA_CTL_SD_LEN0, 0x3FFF_0000);
        assert_eq!(MT_DMA_MAGIC_MASK, 0xF000_0000);
    }

    #[test]
    fn test_descriptor_size() {
        assert_eq!(core::mem::size_of::<Mt76Desc>(), 16);
        assert_eq!(core::mem::align_of::<Mt76Desc>(), 4);
    }

    #[test]
    fn test_tx_descriptor() {
        let desc = Mt76Desc::new_tx(0x1234_5678_9ABC_DEF0, 256);
        assert_eq!(desc.buf0, 0x9ABC_DEF0);
        assert_eq!(desc.buf1, 0x1234_5678);
        assert_eq!(desc.sd_len0(), 256);
        assert!(desc.is_last_sec0());
        assert!(!desc.is_dma_done());
    }

    #[test]
    fn test_magic_cnt() {
        let mut desc = Mt76Desc::new();
        desc.set_magic_cnt(5);
        assert_eq!(desc.magic_cnt(), 5);
        assert_eq!(desc.info, 0x5000_0000);
    }
}
