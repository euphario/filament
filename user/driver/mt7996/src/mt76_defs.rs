//! MT76 Core Definitions
//!
//! Direct translation of mt76/mt76.h from Linux.
//! Contains queue IDs, enums, and core constants.
//!
//! IMPORTANT: Do not modify - keep 1:1 with Linux.
//!
//! Source: https://github.com/openwrt/mt76/blob/master/mt76.h

#![allow(dead_code)]

use super::regs::{bit, genmask};

// ============================================================================
// Ring/Buffer Sizes
// ============================================================================

/// MCU ring size
/// From mt76.h: #define MT_MCU_RING_SIZE 32
pub const MT_MCU_RING_SIZE: u32 = 32;

/// RX buffer size
/// From mt76.h: #define MT_RX_BUF_SIZE 2048
pub const MT_RX_BUF_SIZE: u32 = 2048;

/// SKB head length
/// From mt76.h: #define MT_SKB_HEAD_LEN 256
pub const MT_SKB_HEAD_LEN: u32 = 256;

/// Maximum non-AQL packets
/// From mt76.h: #define MT_MAX_NON_AQL_PKT 16
pub const MT_MAX_NON_AQL_PKT: u32 = 16;

/// TXQ free threshold
/// From mt76.h: #define MT_TXQ_FREE_THR 32
pub const MT_TXQ_FREE_THR: u32 = 32;

/// Token free threshold
/// From mt76.h: #define MT76_TOKEN_FREE_THR 64
pub const MT76_TOKEN_FREE_THR: u32 = 64;

// ============================================================================
// WED Constants
// ============================================================================

/// WED WDS minimum
pub const MT76_WED_WDS_MIN: u32 = 256;

/// WED WDS maximum
pub const MT76_WED_WDS_MAX: u32 = 272;

// ============================================================================
// Queue Flags
// ============================================================================

/// Queue flag: WED ring (bits 1:0)
pub const MT_QFLAG_WED_RING: u32 = genmask(1, 0);

/// Queue flag: WED type (bits 4:2)
pub const MT_QFLAG_WED_TYPE: u32 = genmask(4, 2);

/// Queue flag: WED enabled (bit 5)
pub const MT_QFLAG_WED: u32 = bit(5);

/// Queue flag: WED RRO (bit 6)
pub const MT_QFLAG_WED_RRO: u32 = bit(6);

/// Queue flag: WED RRO enabled (bit 7)
pub const MT_QFLAG_WED_RRO_EN: u32 = bit(7);

/// Queue flag: EMI enabled (bit 8)
pub const MT_QFLAG_EMI_EN: u32 = bit(8);

/// Queue flag: NPU (bit 9)
pub const MT_QFLAG_NPU: u32 = bit(9);

// ============================================================================
// WED Queue Type Enum
// ============================================================================

/// WED queue types
/// From mt76.h: enum mt76_wed_type
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt76WedType {
    QTx = 0,
    QTxfree = 1,
    QRx = 2,
    RroQData = 3,
    RroQMsduPg = 4,
    RroQInd = 5,
    RroQRxdmadC = 6,
}

// ============================================================================
// Hardware RRO Mode
// ============================================================================

/// Hardware RRO mode
/// From mt76.h: enum mt76_hwrro_mode
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt76HwrroMode {
    Off = 0,
    V3 = 1,
    V3_1 = 2,
}

// ============================================================================
// TX Queue IDs
// ============================================================================

/// TX queue IDs (per-band)
/// From mt76.h: enum mt76_txq_id
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt76TxqId {
    /// Voice (highest priority)
    Vo = 0, // IEEE80211_AC_VO
    /// Video
    Vi = 1, // IEEE80211_AC_VI
    /// Best effort
    Be = 2, // IEEE80211_AC_BE
    /// Background (lowest priority)
    Bk = 3, // IEEE80211_AC_BK
    /// Power save
    Psd = 4,
    /// Beacon
    Beacon = 5,
    /// CAB (Content After Beacon)
    Cab = 6,
}

/// Maximum TX queue count
pub const MT_TXQ_MAX: usize = 7;

// ============================================================================
// MCU Queue IDs
// ============================================================================

/// MCU queue IDs
/// From mt76.h: enum mt76_mcuq_id
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt76McuqId {
    /// WM (WiFi Manager)
    Wm = 0,
    /// WA (WiFi Agent)
    Wa = 1,
    /// FWDL (Firmware Download)
    Fwdl = 2,
}

/// Maximum MCU queue count
pub const MT_MCUQ_MAX: usize = 3;

// ============================================================================
// RX Queue IDs
// ============================================================================

/// RX queue IDs
/// From mt76.h: enum mt76_rxq_id
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt76RxqId {
    /// Main data RX
    Main = 0,
    /// MCU RX
    Mcu = 1,
    /// MCU WA RX
    McuWa = 2,
    /// Band 1 data RX
    Band1 = 3,
    /// Band 1 WA RX
    Band1Wa = 4,
    /// Main WA RX
    MainWa = 5,
    /// Band 2 data RX
    Band2 = 6,
    /// Band 2 WA RX
    Band2Wa = 7,
    /// RRO Band 0
    RroBand0 = 8,
    /// RRO Band 1
    RroBand1 = 9,
    /// RRO Band 2
    RroBand2 = 10,
    /// MSDU page Band 0
    MsduPageBand0 = 11,
    /// MSDU page Band 1
    MsduPageBand1 = 12,
    /// MSDU page Band 2
    MsduPageBand2 = 13,
    /// TX free Band 0
    TxfreeBand0 = 14,
    /// TX free Band 1
    TxfreeBand1 = 15,
    /// TX free Band 2
    TxfreeBand2 = 16,
    /// RRO indication
    RroInd = 17,
    /// RRO RXDMAD C
    RroRxdmadC = 18,
    /// NPU 0
    Npu0 = 19,
    /// NPU 1
    Npu1 = 20,
}

/// Maximum RX queue count
pub const MT_RXQ_MAX: usize = 21;

// ============================================================================
// Band IDs
// ============================================================================

/// Band IDs
/// From mt76.h: enum mt76_band_id
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt76BandId {
    Band0 = 0,
    Band1 = 1,
    Band2 = 2,
}

/// Maximum band count
pub const MT_MAX_BAND: usize = 3;

// ============================================================================
// Cipher Types
// ============================================================================

/// Cipher types
/// From mt76.h: enum mt76_cipher_type
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt76CipherType {
    None = 0,
    Wep40 = 1,
    Tkip = 2,
    TkipNoMic = 3,
    AesCcmp = 4,
    Wep104 = 5,
    BipCmac128 = 6,
    Wep128 = 7,
    Wapi = 8,
    CcmpCcx = 9,
    Ccmp256 = 10,
    Gcmp = 11,
    Gcmp256 = 12,
}

// ============================================================================
// DFS State
// ============================================================================

/// DFS state
/// From mt76.h: enum mt76_dfs_state
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt76DfsState {
    Unknown = 0,
    Disabled = 1,
    Cac = 2,
    Active = 3,
}

// ============================================================================
// PHY Types
// ============================================================================

/// PHY types
/// From mt76.h: enum mt76_phy_type
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt76PhyType {
    Cck = 0,
    Ofdm = 1,
    Ht = 2,
    HtGf = 3,
    Vht = 4,
    HeSu = 8,
    HeExtSu = 9,
    HeTb = 10,
    HeMu = 11,
    EhtSu = 13,
    EhtTrig = 14,
    EhtMu = 15,
}

/// Maximum PHY type count
pub const MT_PHY_TYPE_MAX: usize = 16;

// ============================================================================
// WCID (Wireless Client ID) Constants
// ============================================================================

/// Maximum number of WCIDs
pub const MT76_N_WCIDS: usize = 1088;

// ============================================================================
// TX Info Fields (stored in ieee80211_tx_info::hw_queue)
// ============================================================================

/// TX HW queue PHY field
pub const MT_TX_HW_QUEUE_PHY: u32 = genmask(3, 2);

// ============================================================================
// WCID TX Info Fields
// ============================================================================

/// WCID TX info rate (bits 15:0)
pub const MT_WCID_TX_INFO_RATE: u32 = genmask(15, 0);

/// WCID TX info NSS (bits 17:16)
pub const MT_WCID_TX_INFO_NSS: u32 = genmask(17, 16);

/// WCID TX info TX power adjust (bits 25:18)
pub const MT_WCID_TX_INFO_TXPWR_ADJ: u32 = genmask(25, 18);

/// WCID TX info set flag (bit 31)
pub const MT_WCID_TX_INFO_SET: u32 = bit(31);

// ============================================================================
// RRO Indication Data Fields
// ============================================================================

/// RRO indication data0: indication reason mask (bits 31:28)
pub const RRO_IND_DATA0_IND_REASON_MASK: u32 = genmask(31, 28);

/// RRO indication data0: start sequence mask (bits 27:16)
pub const RRO_IND_DATA0_START_SEQ_MASK: u32 = genmask(27, 16);

/// RRO indication data0: sequence ID mask (bits 11:0)
pub const RRO_IND_DATA0_SEQ_ID_MASK: u32 = genmask(11, 0);

/// RRO indication data1: magic counter mask (bits 31:29)
pub const RRO_IND_DATA1_MAGIC_CNT_MASK: u32 = genmask(31, 29);

/// RRO indication data1: indication count mask (bits 12:0)
pub const RRO_IND_DATA1_IND_COUNT_MASK: u32 = genmask(12, 0);

// ============================================================================
// TX CB (Control Block) Flags
// ============================================================================

/// TX CB DMA done flag
pub const MT_TX_CB_DMA_DONE: u32 = bit(0);

/// TX CB TXS done flag
pub const MT_TX_CB_TXS_DONE: u32 = bit(1);

/// TX CB TXS failed flag
pub const MT_TX_CB_TXS_FAILED: u32 = bit(2);

// ============================================================================
// Packet ID Constants
// ============================================================================

/// Packet ID mask
pub const MT_PACKET_ID_MASK: u32 = genmask(6, 0);

/// Packet ID: no ACK
pub const MT_PACKET_ID_NO_ACK: u32 = 0;

/// Packet ID: no SKB
pub const MT_PACKET_ID_NO_SKB: u32 = 1;

/// Packet ID: WED
pub const MT_PACKET_ID_WED: u32 = 2;

/// Packet ID: first
pub const MT_PACKET_ID_FIRST: u32 = 3;

/// Packet ID: has rate flag
pub const MT_PACKET_ID_HAS_RATE: u32 = bit(7);

// ============================================================================
// Driver Flags
// ============================================================================

/// Driver flag: TXWI no free
pub const MT_DRV_TXWI_NO_FREE: u32 = bit(0);

/// Driver flag: TX aligned 4 SKBs
pub const MT_DRV_TX_ALIGNED4_SKBS: u32 = bit(1);

/// Driver flag: SW RX airtime
pub const MT_DRV_SW_RX_AIRTIME: u32 = bit(2);

/// Driver flag: RX DMA header
pub const MT_DRV_RX_DMA_HDR: u32 = bit(3);

/// Driver flag: HW management TX queue
pub const MT_DRV_HW_MGMT_TXQ: u32 = bit(4);

/// Driver flag: AMSDU offload
pub const MT_DRV_AMSDU_OFFLOAD: u32 = bit(5);

/// Driver flag: Ignore TXS failed
pub const MT_DRV_IGNORE_TXS_FAILED: u32 = bit(6);

// ============================================================================
// State Flags
// ============================================================================

/// Device state flags
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt76State {
    Initialized = 0,
    Registered = 1,
    Running = 2,
    McuRunning = 3,
    Scanning = 4,
    HwScanning = 5,
    HwSchedScanning = 6,
    Restart = 7,
    Reset = 8,
    McuReset = 9,
    Removed = 10,
    ReadingStats = 11,
    PowerOff = 12,
    Suspend = 13,
    Roc = 14,
    Pm = 15,
    WedReset = 16,
}

// ============================================================================
// STA Events
// ============================================================================

/// Station events
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt76StaEvent {
    Assoc = 0,
    Authorize = 1,
    Disassoc = 2,
}

// ============================================================================
// Vendor Types
// ============================================================================

/// Vendor type EEPROM flag
pub const MT_VEND_TYPE_EEPROM: u32 = bit(31);

/// Vendor type CFG flag
pub const MT_VEND_TYPE_CFG: u32 = bit(30);

/// Vendor type mask
pub const MT_VEND_TYPE_MASK: u32 = MT_VEND_TYPE_EEPROM | MT_VEND_TYPE_CFG;

// ============================================================================
// Queue Register Structure
// ============================================================================

/// Queue register offsets (within each ring's register block)
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Mt76QueueRegs {
    /// Descriptor base address
    pub desc_base: u32,
    /// Ring size
    pub ring_size: u32,
    /// CPU index
    pub cpu_idx: u32,
    /// DMA index
    pub dma_idx: u32,
}

impl Mt76QueueRegs {
    /// Offset of desc_base field
    pub const DESC_BASE_OFFSET: u32 = 0x00;
    /// Offset of ring_size field
    pub const RING_SIZE_OFFSET: u32 = 0x04;
    /// Offset of cpu_idx field
    pub const CPU_IDX_OFFSET: u32 = 0x08;
    /// Offset of dma_idx field
    pub const DMA_IDX_OFFSET: u32 = 0x0C;
}

// ============================================================================
// Queue Buffer
// ============================================================================

/// Queue buffer descriptor
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Mt76QueueBuf {
    /// DMA address
    pub addr: u64,
    /// Length (15 bits) and skip_unmap flag (1 bit)
    pub len_flags: u16,
}

impl Mt76QueueBuf {
    /// Length mask (bits 14:0)
    pub const LEN_MASK: u16 = 0x7FFF;
    /// Skip unmap flag (bit 15)
    pub const SKIP_UNMAP: u16 = 0x8000;

    /// Get length
    pub fn len(&self) -> u16 {
        self.len_flags & Self::LEN_MASK
    }

    /// Check if skip_unmap is set
    pub fn skip_unmap(&self) -> bool {
        (self.len_flags & Self::SKIP_UNMAP) != 0
    }
}

// ============================================================================
// USB/SDIO Constants (for completeness)
// ============================================================================

/// TX scatter-gather max size
pub const MT_TX_SG_MAX_SIZE: usize = 8;

/// RX scatter-gather max size
pub const MT_RX_SG_MAX_SIZE: usize = 4;

/// Number of TX entries
pub const MT_NUM_TX_ENTRIES: usize = 256;

/// Number of RX entries
pub const MT_NUM_RX_ENTRIES: usize = 128;

/// MCU response URB size
pub const MCU_RESP_URB_SIZE: usize = 1024;

/// SDIO transmit buffer size
pub const MT76S_XMIT_BUF_SZ: usize = 0x3fe00;

/// SDIO number of TX entries
pub const MT76S_NUM_TX_ENTRIES: usize = 256;

/// SDIO number of RX entries
pub const MT76S_NUM_RX_ENTRIES: usize = 512;

// ============================================================================
// WCID Flags
// ============================================================================

/// WCID flag: check PS
pub const MT_WCID_FLAG_CHECK_PS: u32 = 0;

/// WCID flag: PS
pub const MT_WCID_FLAG_PS: u32 = 1;

/// WCID flag: 4-address mode
pub const MT_WCID_FLAG_4ADDR: u32 = 2;

/// WCID flag: header translation
pub const MT_WCID_FLAG_HDR_TRANS: u32 = 3;
