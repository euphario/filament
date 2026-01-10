//! MT7996 MCU Command Definitions
//!
//! Direct translation of mt76/mt7996/mcu.h from Linux.
//! Contains MCU command IDs, event IDs, TLV tags, and protocol structures.
//!
//! IMPORTANT: Do not modify - keep 1:1 with Linux.
//!
//! Source: https://github.com/openwrt/mt76/blob/master/mt7996/mcu.h

#![allow(dead_code)]

use super::regs::{bit, genmask};

// ============================================================================
// MCU Packet/Queue IDs
// ============================================================================

/// MCU packet queue ID helper
/// From mcu.h: #define MCU_PQ_ID(p, q) (((p) << 15) | ((q) << 10))
pub const fn mcu_pq_id(p: u32, q: u32) -> u32 {
    ((p) << 15) | ((q) << 10)
}

/// MCU packet ID
/// From mcu.h: #define MCU_PKT_ID 0xa0
pub const MCU_PKT_ID: u8 = 0xa0;

// ============================================================================
// Firmware Log Destinations
// ============================================================================

/// MCU FW log to WM
/// From mcu.h: MCU_FW_LOG_WM
pub const MCU_FW_LOG_WM: u32 = 0;

/// MCU FW log to WA
/// From mcu.h: MCU_FW_LOG_WA
pub const MCU_FW_LOG_WA: u32 = 1;

/// MCU FW log to host
/// From mcu.h: MCU_FW_LOG_TO_HOST
pub const MCU_FW_LOG_TO_HOST: u32 = 2;

/// MCU FW log relay
/// From mcu.h: MCU_FW_LOG_RELAY = 16
pub const MCU_FW_LOG_RELAY: u32 = 16;

// ============================================================================
// TWT Agreement Commands
// ============================================================================

/// TWT agreement add
pub const MCU_TWT_AGRT_ADD: u32 = 0;

/// TWT agreement modify
pub const MCU_TWT_AGRT_MODIFY: u32 = 1;

/// TWT agreement delete
pub const MCU_TWT_AGRT_DELETE: u32 = 2;

/// TWT agreement teardown
pub const MCU_TWT_AGRT_TEARDOWN: u32 = 3;

/// TWT agreement get TSF
pub const MCU_TWT_AGRT_GET_TSF: u32 = 4;

// ============================================================================
// WA Parameter Commands
// ============================================================================

/// WA param query
pub const MCU_WA_PARAM_CMD_QUERY: u32 = 0;

/// WA param set
pub const MCU_WA_PARAM_CMD_SET: u32 = 1;

/// WA param capability
pub const MCU_WA_PARAM_CMD_CAPABILITY: u32 = 2;

/// WA param debug
pub const MCU_WA_PARAM_CMD_DEBUG: u32 = 3;

// ============================================================================
// WA Parameter Types
// ============================================================================

/// WA param PDMA RX
pub const MCU_WA_PARAM_PDMA_RX: u32 = 0x04;

/// WA param CPU utilization
pub const MCU_WA_PARAM_CPU_UTIL: u32 = 0x0b;

/// WA param RED (Random Early Detection)
pub const MCU_WA_PARAM_RED: u32 = 0x0e;

/// WA param HW path HIF version
pub const MCU_WA_PARAM_HW_PATH_HIF_VER: u32 = 0x2f;

// ============================================================================
// MMPS (MIMO Power Save) Mode
// ============================================================================

/// MMPS mode enum
/// From mcu.h: enum mcu_mmps_mode
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum McuMmpsMode {
    /// Static MMPS
    Static = 0,
    /// Dynamic MMPS
    Dynamic = 1,
    /// Reserved
    Rsv = 2,
    /// MMPS disabled
    Disable = 3,
}

// ============================================================================
// Beamforming Control
// ============================================================================

/// BP (Beamforming Priority) disable
pub const BP_DISABLE: u32 = 0;

/// BP software mode
pub const BP_SW_MODE: u32 = 1;

/// BP hardware mode
pub const BP_HW_MODE: u32 = 2;

// ============================================================================
// Rate Parameters
// ============================================================================

/// Rate param fixed
pub const RATE_PARAM_FIXED: u32 = 3;

/// Rate param MMPS update
pub const RATE_PARAM_MMPS_UPDATE: u32 = 5;

/// Rate param fixed HE LTF
pub const RATE_PARAM_FIXED_HE_LTF: u32 = 7;

/// Rate param fixed MCS
pub const RATE_PARAM_FIXED_MCS: u32 = 8;

/// Rate param fixed GI
pub const RATE_PARAM_FIXED_GI: u32 = 11;

/// Rate param auto
pub const RATE_PARAM_AUTO: u32 = 20;

// ============================================================================
// Beamforming Sounding
// ============================================================================

/// BF sounding on
pub const BF_SOUNDING_ON: u32 = 1;

/// BF HW enable update
pub const BF_HW_EN_UPDATE: u32 = 17;

/// BF mod enable ctrl
pub const BF_MOD_EN_CTRL: u32 = 20;

// ============================================================================
// Command Band Types
// ============================================================================

/// Command band none
pub const CMD_BAND_NONE: u32 = 0;

/// Command band 2.4GHz
pub const CMD_BAND_24G: u32 = 1;

/// Command band 5GHz
pub const CMD_BAND_5G: u32 = 2;

/// Command band 6GHz
pub const CMD_BAND_6G: u32 = 3;

// ============================================================================
// UNI Channel Commands
// ============================================================================

/// UNI channel switch
pub const UNI_CHANNEL_SWITCH: u32 = 0;

/// UNI channel RX path
pub const UNI_CHANNEL_RX_PATH: u32 = 1;

// ============================================================================
// UNI Band Config Commands
// ============================================================================

/// UNI band config radio enable
pub const UNI_BAND_CONFIG_RADIO_ENABLE: u32 = 0;

/// UNI band config RTS threshold
pub const UNI_BAND_CONFIG_RTS_THRESHOLD: u32 = 0x08;

// ============================================================================
// UNI WSYS Config Commands
// ============================================================================

/// UNI WSYS config FW log ctrl
pub const UNI_WSYS_CONFIG_FW_LOG_CTRL: u32 = 0;

/// UNI WSYS config FW debug ctrl
pub const UNI_WSYS_CONFIG_FW_DBG_CTRL: u32 = 1;

// ============================================================================
// UNI RDD (Radar Detection) Commands
// ============================================================================

/// UNI RDD ctrl param
pub const UNI_RDD_CTRL_PARM: u32 = 0;

/// UNI RDD ctrl set threshold
pub const UNI_RDD_CTRL_SET_TH: u32 = 0x3;

// ============================================================================
// UNI EFUSE Commands
// ============================================================================

/// UNI EFUSE access
pub const UNI_EFUSE_ACCESS: u32 = 1;

/// UNI EFUSE buffer mode
pub const UNI_EFUSE_BUFFER_MODE: u32 = 2;

/// UNI EFUSE free block
pub const UNI_EFUSE_FREE_BLOCK: u32 = 3;

/// UNI EFUSE buffer read
pub const UNI_EFUSE_BUFFER_RD: u32 = 4;

// ============================================================================
// UNI VOW (Voice over WiFi) Commands
// ============================================================================

/// UNI VOW DRR ctrl
pub const UNI_VOW_DRR_CTRL: u32 = 0;

/// UNI VOW RX airtime enable
pub const UNI_VOW_RX_AT_AIRTIME_EN: u32 = 0x0b;

/// UNI VOW RX airtime clear enable
pub const UNI_VOW_RX_AT_AIRTIME_CLR_EN: u32 = 0x0e;

// ============================================================================
// UNI Command MIB
// ============================================================================

/// UNI command MIB data
pub const UNI_CMD_MIB_DATA: u32 = 0;

// ============================================================================
// UNI Power Command
// ============================================================================

/// UNI power off
pub const UNI_POWER_OFF: u32 = 0;

// ============================================================================
// UNI TWT Commands
// ============================================================================

/// UNI command TWT agreement update
pub const UNI_CMD_TWT_ARGT_UPDATE: u32 = 0x0;

/// UNI command TWT management offload
pub const UNI_CMD_TWT_MGMT_OFFLOAD: u32 = 1;

// ============================================================================
// UNI RRO Commands
// ============================================================================

/// UNI RRO delete entry
pub const UNI_RRO_DEL_ENTRY: u32 = 0x1;

/// UNI RRO set platform type
pub const UNI_RRO_SET_PLATFORM_TYPE: u32 = 2;

/// UNI RRO get BA session table
pub const UNI_RRO_GET_BA_SESSION_TABLE: u32 = 3;

/// UNI RRO set bypass mode
pub const UNI_RRO_SET_BYPASS_MODE: u32 = 4;

/// UNI RRO set TX free path
pub const UNI_RRO_SET_TXFREE_PATH: u32 = 5;

/// UNI RRO delete BA session
pub const UNI_RRO_DEL_BA_SESSION: u32 = 6;

/// UNI RRO set flush timeout
pub const UNI_RRO_SET_FLUSH_TIMEOUT: u32 = 7;

// ============================================================================
// UNI SR (Spatial Reuse) Commands
// ============================================================================

/// UNI command SR enable
pub const UNI_CMD_SR_ENABLE: u32 = 0x1;

/// UNI command SR enable SD
pub const UNI_CMD_SR_ENABLE_SD: u32 = 2;

/// UNI command SR enable mode
pub const UNI_CMD_SR_ENABLE_MODE: u32 = 3;

/// UNI command SR enable DPD
pub const UNI_CMD_SR_ENABLE_DPD: u32 = 0x12;

/// UNI command SR enable TX
pub const UNI_CMD_SR_ENABLE_TX: u32 = 0x13;

/// UNI command SR set SRG bitmap
pub const UNI_CMD_SR_SET_SRG_BITMAP: u32 = 0x80;

/// UNI command SR set param
pub const UNI_CMD_SR_SET_PARAM: u32 = 0xc1;

/// UNI command SR set SIGA
pub const UNI_CMD_SR_SET_SIGA: u32 = 0xd0;

// ============================================================================
// UNI Thermal Commands
// ============================================================================

/// UNI command thermal protect enable
pub const UNI_CMD_THERMAL_PROTECT_ENABLE: u32 = 0x6;

/// UNI command thermal protect disable
pub const UNI_CMD_THERMAL_PROTECT_DISABLE: u32 = 7;

/// UNI command thermal protect duty config
pub const UNI_CMD_THERMAL_PROTECT_DUTY_CONFIG: u32 = 8;

// ============================================================================
// UNI TX Power Commands
// ============================================================================

/// UNI TX power limit table ctrl
pub const UNI_TXPOWER_POWER_LIMIT_TABLE_CTRL: u32 = 4;

// ============================================================================
// UNI Access Register Commands
// ============================================================================

/// UNI command access register basic
pub const UNI_CMD_ACCESS_REG_BASIC: u32 = 0x0;

/// UNI command access RF register basic
pub const UNI_CMD_ACCESS_RF_REG_BASIC: u32 = 1;

// ============================================================================
// UNI SER (System Error Recovery) Commands
// ============================================================================

/// UNI command SER query
pub const UNI_CMD_SER_QUERY: u32 = 0;

/// UNI command SER set recover L1
pub const UNI_CMD_SER_SET_RECOVER_L1: u32 = 1;

/// UNI command SER set recover L2
pub const UNI_CMD_SER_SET_RECOVER_L2: u32 = 2;

/// UNI command SER set recover L3 RX abort
pub const UNI_CMD_SER_SET_RECOVER_L3_RX_ABORT: u32 = 3;

/// UNI command SER set recover L3 TX abort
pub const UNI_CMD_SER_SET_RECOVER_L3_TX_ABORT: u32 = 4;

/// UNI command SER set recover L3 TX disable
pub const UNI_CMD_SER_SET_RECOVER_L3_TX_DISABLE: u32 = 5;

/// UNI command SER set recover L3 BF
pub const UNI_CMD_SER_SET_RECOVER_L3_BF: u32 = 6;

/// UNI command SER set recover L4 MDP
pub const UNI_CMD_SER_SET_RECOVER_L4_MDP: u32 = 7;

/// UNI command SER set recover from ETH
pub const UNI_CMD_SER_SET_RECOVER_FROM_ETH: u32 = 8;

/// UNI command SER set recover full (alias for L4 MDP)
pub const UNI_CMD_SER_SET_RECOVER_FULL: u32 = 8;

/// UNI command SER set system assert
pub const UNI_CMD_SER_SET_SYSTEM_ASSERT: u32 = 9;

/// UNI command SER enable (action type)
pub const UNI_CMD_SER_ENABLE: u32 = 1;

/// UNI command SER set (action type)
pub const UNI_CMD_SER_SET: u32 = 2;

/// UNI command SER trigger (action type)
pub const UNI_CMD_SER_TRIGGER: u32 = 3;

// ============================================================================
// UNI SDO (Secure Debug Operations) Commands
// ============================================================================

/// UNI command SDO set
pub const UNI_CMD_SDO_SET: u32 = 1;

/// UNI command SDO query
pub const UNI_CMD_SDO_QUERY: u32 = 2;

/// UNI command SDO CP mode
pub const UNI_CMD_SDO_CP_MODE: u32 = 6;

// ============================================================================
// Security Mode
// ============================================================================

/// Security mode enum
/// From mcu.h: MT7996_SEC_MODE_*
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt7996SecMode {
    /// Plain (no encryption)
    Plain = 0,
    /// AES encryption
    Aes = 1,
    /// Scramble encryption
    Scramble = 2,
    /// Max value (marker)
    Max = 3,
}

// ============================================================================
// Patch Security Fields
// ============================================================================

/// Patch security mode mask (bits 31:24)
/// From mcu.h: #define MT7996_PATCH_SEC GENMASK(31, 24)
pub const MT7996_PATCH_SEC: u32 = genmask(31, 24);

/// Patch scramble key mask (bits 15:8)
/// From mcu.h: #define MT7996_PATCH_SCRAMBLE_KEY GENMASK(15, 8)
pub const MT7996_PATCH_SCRAMBLE_KEY: u32 = genmask(15, 8);

/// Patch AES key mask (bits 7:0)
/// From mcu.h: #define MT7996_PATCH_AES_KEY GENMASK(7, 0)
pub const MT7996_PATCH_AES_KEY: u32 = genmask(7, 0);

// ============================================================================
// Security Flags
// ============================================================================

/// Security encrypt flag (bit 0)
/// From mcu.h: #define MT7996_SEC_ENCRYPT BIT(0)
pub const MT7996_SEC_ENCRYPT: u32 = bit(0);

/// Security key index mask (bits 2:1)
/// From mcu.h: #define MT7996_SEC_KEY_IDX GENMASK(2, 1)
pub const MT7996_SEC_KEY_IDX: u32 = genmask(2, 1);

/// Security IV flag (bit 3)
/// From mcu.h: #define MT7996_SEC_IV BIT(3)
pub const MT7996_SEC_IV: u32 = bit(3);

// ============================================================================
// UNI RA (Rate Adaptation) Commands
// ============================================================================

/// UNI RA fixed rate
pub const UNI_RA_FIXED_RATE: u32 = 0xf;

// ============================================================================
// UNI Header Transform Commands
// ============================================================================

/// UNI header transform enable
pub const UNI_HDR_TRANS_EN: u32 = 0;

/// UNI header transform VLAN
pub const UNI_HDR_TRANS_VLAN: u32 = 1;

/// UNI header transform blacklist
pub const UNI_HDR_TRANS_BLACKLIST: u32 = 2;

// ============================================================================
// UNI WED RRO BA Events
// ============================================================================

/// UNI WED RRO BA session status
pub const UNI_WED_RRO_BA_SESSION_STATUS: u32 = 0;

/// UNI WED RRO BA session table
pub const UNI_WED_RRO_BA_SESSION_TBL: u32 = 1;

/// UNI WED RRO BA session delete
pub const UNI_WED_RRO_BA_SESSION_DELETE: u32 = 2;

// ============================================================================
// MIB (Management Information Base) Offsets
// ============================================================================

/// MIB offset for OBSS airtime
/// From mcu.h: UNI_MIB_OBSS_AIRTIME = 26
pub const UNI_MIB_OBSS_AIRTIME: u32 = 26;

/// MIB offset for non-WiFi time
/// From mcu.h: UNI_MIB_NON_WIFI_TIME = 27
pub const UNI_MIB_NON_WIFI_TIME: u32 = 27;

/// MIB offset for TX time
/// From mcu.h: UNI_MIB_TX_TIME = 28
pub const UNI_MIB_TX_TIME: u32 = 28;

/// MIB offset for RX time
/// From mcu.h: UNI_MIB_RX_TIME = 29
pub const UNI_MIB_RX_TIME: u32 = 29;

// ============================================================================
// Size Constants
// ============================================================================

/// Maximum beacon number
/// From mcu.h: #define MAX_BEACON_NUM 32
pub const MAX_BEACON_NUM: usize = 32;

/// Maximum header transform size
pub const MT7996_HDR_TRANS_MAX_SIZE: usize = 24; // sizeof(hdr_trans_en) + sizeof(hdr_trans_vlan) + sizeof(hdr_trans_blacklist)

/// Maximum BSS offload size
pub const MT7996_MAX_BSS_OFFLOAD_SIZE: usize = 2048;

// ============================================================================
// MCU RXD Structure (Response header)
// ============================================================================

/// MCU RX descriptor structure
/// From mcu.h: struct mt7996_mcu_rxd
/// Size: 32 bytes (8 * u32 RXD) + 8 bytes header = 40 bytes total
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct Mt7996McuRxd {
    /// RX descriptor words (8 x u32)
    pub rxd: [u32; 8],
    /// Length
    pub len: u16,
    /// Packet type ID
    pub pkt_type_id: u16,
    /// Event ID
    pub eid: u8,
    /// Sequence
    pub seq: u8,
    /// Option
    pub option: u8,
    /// Reserved
    pub _rsv: u8,
    /// Extended event ID
    pub ext_eid: u8,
    /// Reserved
    pub _rsv1: [u8; 2],
    /// S2D (Source to Destination) index
    pub s2d_index: u8,
}

/// MCU UNI event structure
/// From mcu.h: struct mt7996_mcu_uni_event
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct Mt7996McuUniEvent {
    /// Command ID
    pub cid: u8,
    /// Reserved
    pub _rsv: [u8; 3],
    /// Status (0 = success, others = fail)
    pub status: u32,
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Get patch security mode from patch header value
pub const fn patch_sec_mode(val: u32) -> u32 {
    (val & MT7996_PATCH_SEC) >> 24
}

/// Get patch scramble key from patch header value
pub const fn patch_scramble_key(val: u32) -> u32 {
    (val & MT7996_PATCH_SCRAMBLE_KEY) >> 8
}

/// Get patch AES key from patch header value
pub const fn patch_aes_key(val: u32) -> u32 {
    val & MT7996_PATCH_AES_KEY
}
