//! Connac3 MAC Layer Definitions
//!
//! Direct translation of mt76/mt76_connac3_mac.h from Linux.
//! Contains RXD/TXD bit definitions for the MT7996 family.
//!
//! IMPORTANT: Do not modify - keep 1:1 with Linux.
//!
//! Source: https://github.com/openwrt/mt76/blob/master/mt76_connac3_mac.h

#![allow(dead_code)]

use super::regs::{bit, genmask};
use super::dma_defs::MT_DMA_MAGIC_MASK;

// ============================================================================
// Context/HIF Enums
// ============================================================================

/// Context identifiers
/// From mt76_connac3_mac.h enum (unnamed)
pub const MT_CTX0: u32 = 0;
pub const MT_HIF0: u32 = 0x0;

/// LMAC AC queue identifiers
pub const MT_LMAC_AC00: u32 = 0x0;
pub const MT_LMAC_AC01: u32 = 0x1;
pub const MT_LMAC_AC02: u32 = 0x2;
pub const MT_LMAC_AC03: u32 = 0x3;
pub const MT_LMAC_ALTX0: u32 = 0x10;
pub const MT_LMAC_BMC0: u32 = 0x11;
pub const MT_LMAC_BCN0: u32 = 0x12;
pub const MT_LMAC_PSMP0: u32 = 0x13;

// ============================================================================
// CT (Cut-Through) Constants
// ============================================================================

/// CT parse length
pub const MT_CT_PARSE_LEN: u32 = 72;
/// CT DMA buffer number
pub const MT_CT_DMA_BUF_NUM: u32 = 2;

// ============================================================================
// RXD DW0 - Receive Descriptor Word 0
// ============================================================================

/// Length field (bits 15:0)
pub const MT_RXD0_LENGTH: u32 = genmask(15, 0);

/// Packet flag (bits 19:16)
pub const MT_RXD0_PKT_FLAG: u32 = genmask(19, 16);

/// Packet type (bits 31:27)
pub const MT_RXD0_PKT_TYPE: u32 = genmask(31, 27);

/// Mesh flag (bit 18)
pub const MT_RXD0_MESH: u32 = bit(18);

/// MHCP flag (bit 19)
pub const MT_RXD0_MHCP: u32 = bit(19);

/// Normal ETH type offset (bits 22:16)
pub const MT_RXD0_NORMAL_ETH_TYPE_OFS: u32 = genmask(22, 16);

/// Software packet type mask (bits 31:16)
pub const MT_RXD0_SW_PKT_TYPE_MASK: u32 = genmask(31, 16);

/// Software packet type map value
pub const MT_RXD0_SW_PKT_TYPE_MAP: u32 = 0x380F;

/// Software packet type frame value
pub const MT_RXD0_SW_PKT_TYPE_FRAME: u32 = 0x3801;

// ============================================================================
// RXD DW1 - Receive Descriptor Word 1
// ============================================================================

/// Normal WLAN index (bits 11:0)
pub const MT_RXD1_NORMAL_WLAN_IDX: u32 = genmask(11, 0);

/// Group 1 present flag (bit 16)
pub const MT_RXD1_NORMAL_GROUP_1: u32 = bit(16);

/// Group 2 present flag (bit 17)
pub const MT_RXD1_NORMAL_GROUP_2: u32 = bit(17);

/// Group 3 present flag (bit 18)
pub const MT_RXD1_NORMAL_GROUP_3: u32 = bit(18);

/// Group 4 present flag (bit 19)
pub const MT_RXD1_NORMAL_GROUP_4: u32 = bit(19);

/// Group 5 present flag (bit 20)
pub const MT_RXD1_NORMAL_GROUP_5: u32 = bit(20);

/// Key ID (bits 22:21)
pub const MT_RXD1_NORMAL_KEY_ID: u32 = genmask(22, 21);

/// CM flag (bit 23)
pub const MT_RXD1_NORMAL_CM: u32 = bit(23);

/// CLM flag (bit 24)
pub const MT_RXD1_NORMAL_CLM: u32 = bit(24);

/// ICV error flag (bit 25)
pub const MT_RXD1_NORMAL_ICV_ERR: u32 = bit(25);

/// TKIP MIC error flag (bit 26)
pub const MT_RXD1_NORMAL_TKIP_MIC_ERR: u32 = bit(26);

/// Band index (bits 28:27)
pub const MT_RXD1_NORMAL_BAND_IDX: u32 = genmask(28, 27);

/// SPP enable flag (bit 29)
pub const MT_RXD1_NORMAL_SPP_EN: u32 = bit(29);

/// ADD OM flag (bit 30)
pub const MT_RXD1_NORMAL_ADD_OM: u32 = bit(30);

/// SEC done flag (bit 31)
pub const MT_RXD1_NORMAL_SEC_DONE: u32 = bit(31);

// ============================================================================
// RXD DW2 - Receive Descriptor Word 2
// ============================================================================

/// BSSID (bits 5:0)
pub const MT_RXD2_NORMAL_BSSID: u32 = genmask(5, 0);

/// MAC header length (bits 12:8)
pub const MT_RXD2_NORMAL_MAC_HDR_LEN: u32 = genmask(12, 8);

/// Header translation flag (bit 7)
pub const MT_RXD2_NORMAL_HDR_TRANS: u32 = bit(7);

/// Header offset (bits 15:13)
pub const MT_RXD2_NORMAL_HDR_OFFSET: u32 = genmask(15, 13);

/// Security mode (bits 20:16)
pub const MT_RXD2_NORMAL_SEC_MODE: u32 = genmask(20, 16);

/// MU BAR flag (bit 21)
pub const MT_RXD2_NORMAL_MU_BAR: u32 = bit(21);

/// Software bit (bit 22)
pub const MT_RXD2_NORMAL_SW_BIT: u32 = bit(22);

/// AMSDU error flag (bit 23)
pub const MT_RXD2_NORMAL_AMSDU_ERR: u32 = bit(23);

/// Max length error flag (bit 24)
pub const MT_RXD2_NORMAL_MAX_LEN_ERROR: u32 = bit(24);

/// Header translation error flag (bit 25)
pub const MT_RXD2_NORMAL_HDR_TRANS_ERROR: u32 = bit(25);

/// Internal frame flag (bit 26)
pub const MT_RXD2_NORMAL_INT_FRAME: u32 = bit(26);

/// Fragment flag (bit 27)
pub const MT_RXD2_NORMAL_FRAG: u32 = bit(27);

/// NULL frame flag (bit 28)
pub const MT_RXD2_NORMAL_NULL_FRAME: u32 = bit(28);

/// No data flag (bit 29)
pub const MT_RXD2_NORMAL_NDATA: u32 = bit(29);

/// Non-AMPDU flag (bit 30)
pub const MT_RXD2_NORMAL_NON_AMPDU: u32 = bit(30);

/// BF report flag (bit 31)
pub const MT_RXD2_NORMAL_BF_REPORT: u32 = bit(31);

// ============================================================================
// RXD DW3 - Receive Descriptor Word 3
// ============================================================================

/// RXV sequence (bits 7:0)
pub const MT_RXD3_NORMAL_RXV_SEQ: u32 = genmask(7, 0);

/// Channel frequency (bits 15:8)
pub const MT_RXD3_NORMAL_CH_FREQ: u32 = genmask(15, 8);

/// Address type (bits 17:16)
pub const MT_RXD3_NORMAL_ADDR_TYPE: u32 = genmask(17, 16);

/// U2M (unicast to me) flag (bit 0)
pub const MT_RXD3_NORMAL_U2M: u32 = bit(0);

/// HTC valid flag (bit 18)
pub const MT_RXD3_NORMAL_HTC_VLD: u32 = bit(18);

/// Beacon multicast flag (bit 20)
pub const MT_RXD3_NORMAL_BEACON_MC: u32 = bit(20);

/// Beacon unicast flag (bit 21)
pub const MT_RXD3_NORMAL_BEACON_UC: u32 = bit(21);

/// CO antenna flag (bit 22)
pub const MT_RXD3_NORMAL_CO_ANT: u32 = bit(22);

/// FCS error flag (bit 24)
pub const MT_RXD3_NORMAL_FCS_ERR: u32 = bit(24);

/// IP checksum flag (bit 26)
pub const MT_RXD3_NORMAL_IP_SUM: u32 = bit(26);

/// UDP/TCP checksum flag (bit 27)
pub const MT_RXD3_NORMAL_UDP_TCP_SUM: u32 = bit(27);

/// VLAN to ETH flag (bit 31)
pub const MT_RXD3_NORMAL_VLAN2ETH: u32 = bit(31);

// ============================================================================
// RXD DW4 - Receive Descriptor Word 4
// ============================================================================

/// Payload format (bits 1:0)
pub const MT_RXD4_NORMAL_PAYLOAD_FORMAT: u32 = genmask(1, 0);

/// First AMSDU frame indicator
pub const MT_RXD4_FIRST_AMSDU_FRAME: u32 = genmask(1, 0);

/// Mid AMSDU frame indicator
pub const MT_RXD4_MID_AMSDU_FRAME: u32 = bit(1);

/// Last AMSDU frame indicator
pub const MT_RXD4_LAST_AMSDU_FRAME: u32 = bit(0);

// ============================================================================
// RXV Header
// ============================================================================

/// Band index in RXV header (bit 24)
pub const MT_RXV_HDR_BAND_IDX: u32 = bit(24);

// ============================================================================
// RXD GROUP4 (DW8-11)
// ============================================================================

/// Frame control (bits 15:0 of DW8)
pub const MT_RXD8_FRAME_CONTROL: u32 = genmask(15, 0);

/// Sequence control (bits 15:0 of DW10)
pub const MT_RXD10_SEQ_CTRL: u32 = genmask(15, 0);

/// QoS control (bits 31:16 of DW10)
pub const MT_RXD10_QOS_CTL: u32 = genmask(31, 16);

/// HT control (bits 31:0 of DW11)
pub const MT_RXD11_HT_CONTROL: u32 = genmask(31, 0);

// ============================================================================
// P-RXV (Per-packet RX Vector)
// ============================================================================

/// TX rate (bits 6:0)
pub const MT_PRXV_TX_RATE: u32 = genmask(6, 0);

/// TX DCM flag (bit 4)
pub const MT_PRXV_TX_DCM: u32 = bit(4);

/// TX ER SU 106T flag (bit 5)
pub const MT_PRXV_TX_ER_SU_106T: u32 = bit(5);

/// NSTS (bits 10:7)
pub const MT_PRXV_NSTS: u32 = genmask(10, 7);

/// TXBF flag (bit 11)
pub const MT_PRXV_TXBF: u32 = bit(11);

/// HT AD code flag (bit 12)
pub const MT_PRXV_HT_AD_CODE: u32 = bit(12);

/// HE RU allocation (bits 30:22)
pub const MT_PRXV_HE_RU_ALLOC: u32 = genmask(30, 22);

/// RCPI3 (bits 31:24)
pub const MT_PRXV_RCPI3: u32 = genmask(31, 24);

/// RCPI2 (bits 23:16)
pub const MT_PRXV_RCPI2: u32 = genmask(23, 16);

/// RCPI1 (bits 15:8)
pub const MT_PRXV_RCPI1: u32 = genmask(15, 8);

/// RCPI0 (bits 7:0)
pub const MT_PRXV_RCPI0: u32 = genmask(7, 0);

/// HT short GI (bits 4:3)
pub const MT_PRXV_HT_SHORT_GI: u32 = genmask(4, 3);

/// HT STBC (bits 10:9)
pub const MT_PRXV_HT_STBC: u32 = genmask(10, 9);

/// TX mode (bits 14:11)
pub const MT_PRXV_TX_MODE: u32 = genmask(14, 11);

/// Frame mode (bits 2:0)
pub const MT_PRXV_FRAME_MODE: u32 = genmask(2, 0);

/// DCM flag (bit 5)
pub const MT_PRXV_DCM: u32 = bit(5);

// ============================================================================
// C-RXV (Common RX Vector) - HE
// ============================================================================

/// HE number of users (bits 26:20)
pub const MT_CRXV_HE_NUM_USER: u32 = genmask(26, 20);

/// HE LTF size (bits 28:27)
pub const MT_CRXV_HE_LTF_SIZE: u32 = genmask(28, 27);

/// HE LDPC extra symbol (bit 30)
pub const MT_CRXV_HE_LDPC_EXT_SYM: u32 = bit(30);

/// HE PE disambig (bit 1)
pub const MT_CRXV_HE_PE_DISAMBIG: u32 = bit(1);

/// HE uplink (bit 2)
pub const MT_CRXV_HE_UPLINK: u32 = bit(2);

/// HE MU AID (bits 27:17)
pub const MT_CRXV_HE_MU_AID: u32 = genmask(27, 17);

/// HE beam change (bit 29)
pub const MT_CRXV_HE_BEAM_CHNG: u32 = bit(29);

/// HE Doppler (bit 0)
pub const MT_CRXV_HE_DOPPLER: u32 = bit(0);

/// HE BSS color (bits 15:10)
pub const MT_CRXV_HE_BSS_COLOR: u32 = genmask(15, 10);

/// HE TXOP duration (bits 19:17)
pub const MT_CRXV_HE_TXOP_DUR: u32 = genmask(19, 17);

/// HE SR mask (bits 11:8)
pub const MT_CRXV_HE_SR_MASK: u32 = genmask(11, 8);

/// HE SR1 mask (bits 16:12)
pub const MT_CRXV_HE_SR1_MASK: u32 = genmask(16, 12);

/// HE SR2 mask (bits 20:17)
pub const MT_CRXV_HE_SR2_MASK: u32 = genmask(20, 17);

/// HE SR3 mask (bits 24:21)
pub const MT_CRXV_HE_SR3_MASK: u32 = genmask(24, 21);

/// HE RU0 (bits 8:0)
pub const MT_CRXV_HE_RU0: u32 = genmask(8, 0);

/// HE RU1 (bits 17:9)
pub const MT_CRXV_HE_RU1: u32 = genmask(17, 9);

/// HE RU2 (bits 26:18)
pub const MT_CRXV_HE_RU2: u32 = genmask(26, 18);

/// HE RU3 low bits (bits 31:27)
pub const MT_CRXV_HE_RU3_L: u32 = genmask(31, 27);

/// HE RU3 high bits (bits 3:0)
pub const MT_CRXV_HE_RU3_H: u32 = genmask(3, 0);

// ============================================================================
// C-RXV (Common RX Vector) - EHT
// ============================================================================

/// EHT number of users (bits 26:20)
pub const MT_CRXV_EHT_NUM_USER: u32 = genmask(26, 20);

/// EHT LTF size (bits 28:27)
pub const MT_CRXV_EHT_LTF_SIZE: u32 = genmask(28, 27);

/// EHT LDPC extra symbol (bit 30)
pub const MT_CRXV_EHT_LDPC_EXT_SYM: u32 = bit(30);

/// EHT PE disambig (bit 1)
pub const MT_CRXV_EHT_PE_DISAMBIG: u32 = bit(1);

/// EHT uplink (bit 2)
pub const MT_CRXV_EHT_UPLINK: u32 = bit(2);

/// EHT MU AID (bits 27:17)
pub const MT_CRXV_EHT_MU_AID: u32 = genmask(27, 17);

/// EHT beam change (bit 29)
pub const MT_CRXV_EHT_BEAM_CHNG: u32 = bit(29);

/// EHT Doppler (bit 0)
pub const MT_CRXV_EHT_DOPPLER: u32 = bit(0);

/// EHT BSS color (bits 15:10)
pub const MT_CRXV_EHT_BSS_COLOR: u32 = genmask(15, 10);

/// EHT TXOP duration (bits 23:17)
pub const MT_CRXV_EHT_TXOP_DUR: u32 = genmask(23, 17);

/// EHT SR mask (bits 11:8)
pub const MT_CRXV_EHT_SR_MASK: u32 = genmask(11, 8);

/// EHT SR1 mask (bits 15:12)
pub const MT_CRXV_EHT_SR1_MASK: u32 = genmask(15, 12);

/// EHT SR2 mask (bits 19:16)
pub const MT_CRXV_EHT_SR2_MASK: u32 = genmask(19, 16);

/// EHT SR3 mask (bits 23:20)
pub const MT_CRXV_EHT_SR3_MASK: u32 = genmask(23, 20);

/// EHT RU0 (bits 8:0)
pub const MT_CRXV_EHT_RU0: u32 = genmask(8, 0);

/// EHT RU1 (bits 17:9)
pub const MT_CRXV_EHT_RU1: u32 = genmask(17, 9);

/// EHT RU2 (bits 26:18)
pub const MT_CRXV_EHT_RU2: u32 = genmask(26, 18);

/// EHT RU3 low bits (bits 31:27)
pub const MT_CRXV_EHT_RU3_L: u32 = genmask(31, 27);

/// EHT RU3 high bits (bits 3:0)
pub const MT_CRXV_EHT_RU3_H: u32 = genmask(3, 0);

/// EHT SIG MCS (bits 19:18)
pub const MT_CRXV_EHT_SIG_MCS: u32 = genmask(19, 18);

/// EHT LTF symbols (bits 22:20)
pub const MT_CRXV_EHT_LTF_SYM: u32 = genmask(22, 20);

// ============================================================================
// TX Header Format
// ============================================================================

/// TX header format enum
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxHeaderFormat {
    /// 802.3 format
    Format802_3 = 0,
    /// Command format
    Cmd = 1,
    /// 802.11 format
    Format802_11 = 2,
    /// 802.11 extended format
    Format802_11Ext = 3,
}

/// TX packet type enum
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxPktType {
    /// Cut-through
    Ct = 0,
    /// Store-and-forward
    Sf = 1,
    /// Command
    Cmd = 2,
    /// Firmware
    Fw = 3,
}

/// TX port index enum
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxPortIdx {
    /// LMAC port
    Lmac = 0,
    /// MCU port
    Mcu = 1,
}

/// TX MCU port queue indices
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxMcuPortQIdx {
    RxQ0 = 0x20,
    RxQ1 = 0x21,
    RxQ2 = 0x22,
    RxQ3 = 0x23,
    RxFwdl = 0x3e,
}

/// TX management type enum
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxMgntType {
    Normal = 0,
    Timing = 1,
    Addba = 2,
}

/// TX fragment index enum
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxFragIdx {
    None = 0,
    First = 1,
    Mid = 2,
    Last = 3,
}

// ============================================================================
// CT Info Bits
// ============================================================================

/// Apply TXD flag
pub const MT_CT_INFO_APPLY_TXD: u32 = bit(0);

/// Copy host TXD all flag
pub const MT_CT_INFO_COPY_HOST_TXD_ALL: u32 = bit(1);

/// Management frame flag
pub const MT_CT_INFO_MGMT_FRAME: u32 = bit(2);

/// None cipher frame flag
pub const MT_CT_INFO_NONE_CIPHER_FRAME: u32 = bit(3);

/// HSR2 TX flag
pub const MT_CT_INFO_HSR2_TX: u32 = bit(4);

/// From host flag
pub const MT_CT_INFO_FROM_HOST: u32 = bit(7);

// ============================================================================
// TXD Size
// ============================================================================

/// TXD size in bytes (8 DWORDs)
pub const MT_TXD_SIZE: u32 = 8 * 4;

// ============================================================================
// TXD DW0
// ============================================================================

/// Queue index (bits 31:25)
pub const MT_TXD0_Q_IDX: u32 = genmask(31, 25);

/// Packet format (bits 24:23)
pub const MT_TXD0_PKT_FMT: u32 = genmask(24, 23);

/// ETH type offset (bits 22:16)
pub const MT_TXD0_ETH_TYPE_OFFSET: u32 = genmask(22, 16);

/// TX bytes (bits 15:0)
pub const MT_TXD0_TX_BYTES: u32 = genmask(15, 0);

// ============================================================================
// TXD DW1
// ============================================================================

/// Fixed rate flag (bit 31)
pub const MT_TXD1_FIXED_RATE: u32 = bit(31);

/// Own MAC (bits 30:25)
pub const MT_TXD1_OWN_MAC: u32 = genmask(30, 25);

/// TID (bits 24:21)
pub const MT_TXD1_TID: u32 = genmask(24, 21);

/// BIP flag (bit 24)
pub const MT_TXD1_BIP: u32 = bit(24);

/// ETH 802.3 flag (bit 20)
pub const MT_TXD1_ETH_802_3: u32 = bit(20);

/// Header info (bits 20:16)
pub const MT_TXD1_HDR_INFO: u32 = genmask(20, 16);

/// Header format (bits 15:14)
pub const MT_TXD1_HDR_FORMAT: u32 = genmask(15, 14);

/// TGID (bits 13:12)
pub const MT_TXD1_TGID: u32 = genmask(13, 12);

/// WLAN index (bits 11:0)
pub const MT_TXD1_WLAN_IDX: u32 = genmask(11, 0);

// ============================================================================
// TXD DW2
// ============================================================================

/// Power offset (bits 31:26)
pub const MT_TXD2_POWER_OFFSET: u32 = genmask(31, 26);

/// Max TX time (bits 25:16)
pub const MT_TXD2_MAX_TX_TIME: u32 = genmask(25, 16);

/// Fragment (bits 15:14)
pub const MT_TXD2_FRAG: u32 = genmask(15, 14);

/// HTC valid flag (bit 13)
pub const MT_TXD2_HTC_VLD: u32 = bit(13);

/// Duration flag (bit 12)
pub const MT_TXD2_DURATION: u32 = bit(12);

/// Header padding (bits 11:10)
pub const MT_TXD2_HDR_PAD: u32 = genmask(11, 10);

/// RTS flag (bit 9)
pub const MT_TXD2_RTS: u32 = bit(9);

/// Own MAC map (bit 8)
pub const MT_TXD2_OWN_MAC_MAP: u32 = bit(8);

/// BF type (bits 7:6)
pub const MT_TXD2_BF_TYPE: u32 = genmask(7, 6);

/// Frame type (bits 5:4)
pub const MT_TXD2_FRAME_TYPE: u32 = genmask(5, 4);

/// Sub type (bits 3:0)
pub const MT_TXD2_SUB_TYPE: u32 = genmask(3, 0);

// ============================================================================
// TXD DW3
// ============================================================================

/// SN valid flag (bit 31)
pub const MT_TXD3_SN_VALID: u32 = bit(31);

/// PN valid flag (bit 30)
pub const MT_TXD3_PN_VALID: u32 = bit(30);

/// SW power management flag (bit 29)
pub const MT_TXD3_SW_POWER_MGMT: u32 = bit(29);

/// BA disable flag (bit 28)
pub const MT_TXD3_BA_DISABLE: u32 = bit(28);

/// Sequence number (bits 27:16)
pub const MT_TXD3_SEQ: u32 = genmask(27, 16);

/// Remaining TX count (bits 15:11)
pub const MT_TXD3_REM_TX_COUNT: u32 = genmask(15, 11);

/// TX count (bits 10:6)
pub const MT_TXD3_TX_COUNT: u32 = genmask(10, 6);

/// HW AMSDU flag (bit 5)
pub const MT_TXD3_HW_AMSDU: u32 = bit(5);

/// BCM flag (bit 4)
pub const MT_TXD3_BCM: u32 = bit(4);

/// EEOSP flag (bit 3)
pub const MT_TXD3_EEOSP: u32 = bit(3);

/// EMRD flag (bit 2)
pub const MT_TXD3_EMRD: u32 = bit(2);

/// Protect frame flag (bit 1)
pub const MT_TXD3_PROTECT_FRAME: u32 = bit(1);

/// No ACK flag (bit 0)
pub const MT_TXD3_NO_ACK: u32 = bit(0);

// ============================================================================
// TXD DW4
// ============================================================================

/// PN low (bits 31:0)
pub const MT_TXD4_PN_LOW: u32 = genmask(31, 0);

// ============================================================================
// TXD DW5
// ============================================================================

/// PN high (bits 31:16)
pub const MT_TXD5_PN_HIGH: u32 = genmask(31, 16);

/// FL flag (bit 15)
pub const MT_TXD5_FL: u32 = bit(15);

/// Bypass TBB flag (bit 14)
pub const MT_TXD5_BYPASS_TBB: u32 = bit(14);

/// Bypass RBB flag (bit 13)
pub const MT_TXD5_BYPASS_RBB: u32 = bit(13);

/// BSS color zero flag (bit 12)
pub const MT_TXD5_BSS_COLOR_ZERO: u32 = bit(12);

/// TX status to host flag (bit 10)
pub const MT_TXD5_TX_STATUS_HOST: u32 = bit(10);

/// TX status to MCU flag (bit 9)
pub const MT_TXD5_TX_STATUS_MCU: u32 = bit(9);

/// TX status format flag (bit 8)
pub const MT_TXD5_TX_STATUS_FMT: u32 = bit(8);

/// PID (bits 7:0)
pub const MT_TXD5_PID: u32 = genmask(7, 0);

// ============================================================================
// TXD DW6
// ============================================================================

/// TX source (bits 31:30)
pub const MT_TXD6_TX_SRC: u32 = genmask(31, 30);

/// VTA flag (bit 28)
pub const MT_TXD6_VTA: u32 = bit(28);

/// Fixed BW flag (bit 25)
pub const MT_TXD6_FIXED_BW: u32 = bit(25);

/// Bandwidth (bits 24:22)
pub const MT_TXD6_BW: u32 = genmask(24, 22);

/// TX rate (bits 21:16)
pub const MT_TXD6_TX_RATE: u32 = genmask(21, 16);

/// Timestamp offset enable flag (bit 15)
pub const MT_TXD6_TIMESTAMP_OFS_EN: u32 = bit(15);

/// Timestamp offset index (bits 14:10)
pub const MT_TXD6_TIMESTAMP_OFS_IDX: u32 = genmask(14, 10);

/// TID ADDBA (bits 10:8)
pub const MT_TXD6_TID_ADDBA: u32 = genmask(10, 8);

/// MSDU count (bits 9:4)
pub const MT_TXD6_MSDU_CNT: u32 = genmask(9, 4);

/// MSDU count V2 (bits 15:10)
pub const MT_TXD6_MSDU_CNT_V2: u32 = genmask(15, 10);

/// Disable MAT flag (bit 3)
pub const MT_TXD6_DIS_MAT: u32 = bit(3);

/// DAS flag (bit 2)
pub const MT_TXD6_DAS: u32 = bit(2);

/// AMSDU cap flag (bit 1)
pub const MT_TXD6_AMSDU_CAP: u32 = bit(1);

// ============================================================================
// TXD DW7
// ============================================================================

/// TXD length (bits 31:30)
pub const MT_TXD7_TXD_LEN: u32 = genmask(31, 30);

/// IP checksum flag (bit 29)
pub const MT_TXD7_IP_SUM: u32 = bit(29);

/// Drop by SDO flag (bit 28)
pub const MT_TXD7_DROP_BY_SDO: u32 = bit(28);

/// MAC TXD flag (bit 27)
pub const MT_TXD7_MAC_TXD: u32 = bit(27);

/// CTXD flag (bit 26)
pub const MT_TXD7_CTXD: u32 = bit(26);

/// CTXD count (bits 25:22)
pub const MT_TXD7_CTXD_CNT: u32 = genmask(25, 22);

/// UDP/TCP checksum flag (bit 15)
pub const MT_TXD7_UDP_TCP_SUM: u32 = bit(15);

/// TX time (bits 9:0)
pub const MT_TXD7_TX_TIME: u32 = genmask(9, 0);

// ============================================================================
// TXD DW9
// ============================================================================

/// WLAN index (bits 23:8)
pub const MT_TXD9_WLAN_IDX: u32 = genmask(23, 8);

// ============================================================================
// TXP (TX Payload) Fields
// ============================================================================

/// Buffer length (bits 11:0)
pub const MT_TXP_BUF_LEN: u32 = genmask(11, 0);

/// DMA address high bits (bits 15:12)
pub const MT_TXP_DMA_ADDR_H: u32 = genmask(15, 12);

/// Token ID 0 (bits 14:0)
pub const MT_TXP0_TOKEN_ID0: u32 = genmask(14, 0);

/// Token ID 0 valid mask (bit 15)
pub const MT_TXP0_TOKEN_ID0_VALID_MASK: u32 = bit(15);

/// TID ADDBA (bits 14:12)
pub const MT_TXP1_TID_ADDBA: u32 = genmask(14, 12);

/// ML0 mask (bit 15)
pub const MT_TXP3_ML0_MASK: u32 = bit(15);

/// DMA address high (bits 13:12)
pub const MT_TXP3_DMA_ADDR_H: u32 = genmask(13, 12);

// ============================================================================
// TX Rate Fields
// ============================================================================

/// STBC flag (bit 14)
pub const MT_TX_RATE_STBC: u32 = bit(14);

/// NSS (bits 13:10)
pub const MT_TX_RATE_NSS: u32 = genmask(13, 10);

/// Mode (bits 9:6)
pub const MT_TX_RATE_MODE: u32 = genmask(9, 6);

/// SU extended tone flag (bit 5)
pub const MT_TX_RATE_SU_EXT_TONE: u32 = bit(5);

/// DCM flag (bit 4)
pub const MT_TX_RATE_DCM: u32 = bit(4);

/// Rate index (bits 5:0) - VHT/HE only use bits 0-3
pub const MT_TX_RATE_IDX: u32 = genmask(5, 0);

// ============================================================================
// TX Free Fields
// ============================================================================

/// Packet type (bits 31:27)
pub const MT_TXFREE0_PKT_TYPE: u32 = genmask(31, 27);

/// MSDU count (bits 25:16)
pub const MT_TXFREE0_MSDU_CNT: u32 = genmask(25, 16);

/// RX byte (bits 15:0)
pub const MT_TXFREE0_RX_BYTE: u32 = genmask(15, 0);

/// Version (bits 19:16)
pub const MT_TXFREE1_VER: u32 = genmask(19, 16);

/// Pair flag (bit 31)
pub const MT_TXFREE_INFO_PAIR: u32 = bit(31);

/// Header flag (bit 30)
pub const MT_TXFREE_INFO_HEADER: u32 = bit(30);

/// WLAN ID (bits 23:12)
pub const MT_TXFREE_INFO_WLAN_ID: u32 = genmask(23, 12);

/// MSDU ID (bits 14:0)
pub const MT_TXFREE_INFO_MSDU_ID: u32 = genmask(14, 0);

/// Count (bits 27:24)
pub const MT_TXFREE_INFO_COUNT: u32 = genmask(27, 24);

/// Status (bits 29:28)
pub const MT_TXFREE_INFO_STAT: u32 = genmask(29, 28);

// ============================================================================
// TX Status (TXS) Fields
// ============================================================================

/// TXS header size in DWORDs
pub const MT_TXS_HDR_SIZE: u32 = 4;

/// TXS size in DWORDs
pub const MT_TXS_SIZE: u32 = 12;

// TXS DW0
/// Bandwidth (bits 31:29)
pub const MT_TXS0_BW: u32 = genmask(31, 29);

/// TID (bits 28:26)
pub const MT_TXS0_TID: u32 = genmask(28, 26);

/// AMPDU flag (bit 25)
pub const MT_TXS0_AMPDU: u32 = bit(25);

/// TXS format (bits 24:23)
pub const MT_TXS0_TXS_FORMAT: u32 = genmask(24, 23);

/// BA error flag (bit 22)
pub const MT_TXS0_BA_ERROR: u32 = bit(22);

/// PS flag (bit 21)
pub const MT_TXS0_PS_FLAG: u32 = bit(21);

/// TXOP timeout flag (bit 20)
pub const MT_TXS0_TXOP_TIMEOUT: u32 = bit(20);

/// BIP error flag (bit 19)
pub const MT_TXS0_BIP_ERROR: u32 = bit(19);

/// Queue timeout flag (bit 18)
pub const MT_TXS0_QUEUE_TIMEOUT: u32 = bit(18);

/// RTS timeout flag (bit 17)
pub const MT_TXS0_RTS_TIMEOUT: u32 = bit(17);

/// ACK timeout flag (bit 16)
pub const MT_TXS0_ACK_TIMEOUT: u32 = bit(16);

/// ACK error mask (bits 18:16)
pub const MT_TXS0_ACK_ERROR_MASK: u32 = genmask(18, 16);

/// TX status to host flag (bit 15)
pub const MT_TXS0_TX_STATUS_HOST: u32 = bit(15);

/// TX status to MCU flag (bit 14)
pub const MT_TXS0_TX_STATUS_MCU: u32 = bit(14);

/// TX rate (bits 13:0)
pub const MT_TXS0_TX_RATE: u32 = genmask(13, 0);

// TXS DW1
/// Sequence number (bits 31:20)
pub const MT_TXS1_SEQNO: u32 = genmask(31, 20);

/// Response rate (bits 19:16)
pub const MT_TXS1_RESP_RATE: u32 = genmask(19, 16);

/// RXV sequence number (bits 15:8)
pub const MT_TXS1_RXV_SEQNO: u32 = genmask(15, 8);

/// TX power in dBm (bits 7:0)
pub const MT_TXS1_TX_POWER_DBM: u32 = genmask(7, 0);

// TXS DW2
/// BF status (bits 31:30)
pub const MT_TXS2_BF_STATUS: u32 = genmask(31, 30);

/// Band (bits 29:28)
pub const MT_TXS2_BAND: u32 = genmask(29, 28);

/// WCID (bits 27:16)
pub const MT_TXS2_WCID: u32 = genmask(27, 16);

/// TX delay (bits 15:0)
pub const MT_TXS2_TX_DELAY: u32 = genmask(15, 0);

// TXS DW3
/// PID (bits 31:24)
pub const MT_TXS3_PID: u32 = genmask(31, 24);

/// Rate STBC flag (bit 7)
pub const MT_TXS3_RATE_STBC: u32 = bit(7);

/// Fixed rate flag (bit 6)
pub const MT_TXS3_FIXED_RATE: u32 = bit(6);

/// Source (bits 5:4)
pub const MT_TXS3_SRC: u32 = genmask(5, 4);

/// Shared antenna flag (bit 3)
pub const MT_TXS3_SHARED_ANTENNA: u32 = bit(3);

/// Last TX rate (bits 2:0)
pub const MT_TXS3_LAST_TX_RATE: u32 = genmask(2, 0);

// TXS DW4
/// Timestamp (bits 31:0)
pub const MT_TXS4_TIMESTAMP: u32 = genmask(31, 0);

// TXS DW5 - MPDU based
/// Final MPDU flag (bit 31)
pub const MT_TXS5_F0_FINAL_MPDU: u32 = bit(31);

/// QoS flag (bit 30)
pub const MT_TXS5_F0_QOS: u32 = bit(30);

/// TX count (bits 29:25)
pub const MT_TXS5_F0_TX_COUNT: u32 = genmask(29, 25);

/// Front time (bits 24:0)
pub const MT_TXS5_F0_FRONT_TIME: u32 = genmask(24, 0);

/// MPDU TX count (bits 31:24)
pub const MT_TXS5_F1_MPDU_TX_COUNT: u32 = genmask(31, 24);

/// MPDU TX bytes (bits 23:0)
pub const MT_TXS5_F1_MPDU_TX_BYTES: u32 = genmask(23, 0);

// TXS DW6 - MPDU based
/// Noise 3 (bits 31:24)
pub const MT_TXS6_F0_NOISE_3: u32 = genmask(31, 24);

/// Noise 2 (bits 23:16)
pub const MT_TXS6_F0_NOISE_2: u32 = genmask(23, 16);

/// Noise 1 (bits 15:8)
pub const MT_TXS6_F0_NOISE_1: u32 = genmask(15, 8);

/// Noise 0 (bits 7:0)
pub const MT_TXS6_F0_NOISE_0: u32 = genmask(7, 0);

/// MPDU fail count (bits 31:24)
pub const MT_TXS6_F1_MPDU_FAIL_COUNT: u32 = genmask(31, 24);

/// MPDU fail bytes (bits 23:0)
pub const MT_TXS6_F1_MPDU_FAIL_BYTES: u32 = genmask(23, 0);

// TXS DW7 - MPDU based
/// RCPI 3 (bits 31:24)
pub const MT_TXS7_F0_RCPI_3: u32 = genmask(31, 24);

/// RCPI 2 (bits 23:16)
pub const MT_TXS7_F0_RCPI_2: u32 = genmask(23, 16);

/// RCPI 1 (bits 15:8)
pub const MT_TXS7_F0_RCPI_1: u32 = genmask(15, 8);

/// RCPI 0 (bits 7:0)
pub const MT_TXS7_F0_RCPI_0: u32 = genmask(7, 0);

/// MPDU retry count (bits 31:24)
pub const MT_TXS7_F1_MPDU_RETRY_COUNT: u32 = genmask(31, 24);

/// MPDU retry bytes (bits 23:0)
pub const MT_TXS7_F1_MPDU_RETRY_BYTES: u32 = genmask(23, 0);

// TXS DW5-7 - PPDU based
/// MPDU TX count (bits 30:20)
pub const MT_TXS5_MPDU_TX_CNT: u32 = genmask(30, 20);

/// MPDU TX byte scale flag (bit 15)
pub const MT_TXS5_MPDU_TX_BYTE_SCALE: u32 = bit(15);

/// MPDU TX byte (bits 14:0)
pub const MT_TXS5_MPDU_TX_BYTE: u32 = genmask(14, 0);

/// MPDU fail count (bits 30:20)
pub const MT_TXS6_MPDU_FAIL_CNT: u32 = genmask(30, 20);

/// MPDU fail byte scale flag (bit 15)
pub const MT_TXS6_MPDU_FAIL_BYTE_SCALE: u32 = bit(15);

/// MPDU fail byte (bits 14:0)
pub const MT_TXS6_MPDU_FAIL_BYTE: u32 = genmask(14, 0);

/// MPDU retry count (bits 30:20)
pub const MT_TXS7_MPDU_RETRY_CNT: u32 = genmask(30, 20);

/// MPDU retry byte scale flag (bit 15)
pub const MT_TXS7_MPDU_RETRY_BYTE_SCALE: u32 = bit(15);

/// MPDU retry byte (bits 14:0)
pub const MT_TXS7_MPDU_RETRY_BYTE: u32 = genmask(14, 0);

// ============================================================================
// Validation
// ============================================================================

/// Validate key mask values against expected Linux values
pub fn validate_masks() {
    debug_assert_eq!(MT_DMA_MAGIC_MASK, 0xF000_0000, "MAGIC_MASK must be bits 31:28");
    debug_assert_eq!(MT_RXD0_LENGTH, 0x0000_FFFF, "RXD0_LENGTH mismatch");
    debug_assert_eq!(MT_TXD_SIZE, 32, "TXD_SIZE should be 32 bytes");
}
