//! MT7996 Device-Specific Definitions
//!
//! Direct translation of mt76/mt7996/mt7996.h from Linux.
//! Contains device IDs, ring sizes, queue mappings, and device constants.
//!
//! IMPORTANT: Do not modify - keep 1:1 with Linux.
//!
//! Source: https://github.com/openwrt/mt76/blob/master/mt7996/mt7996.h

#![allow(dead_code)]

use super::regs::{bit, genmask};

// ============================================================================
// Device Constants
// ============================================================================

/// Maximum number of radios
pub const MT7996_MAX_RADIOS: usize = 3;

/// Maximum interfaces per band
pub const MT7996_MAX_INTERFACES: usize = 19;

/// Maximum WMM sets
pub const MT7996_MAX_WMM_SETS: usize = 4;

// ============================================================================
// Timing Constants
// ============================================================================

/// Watchdog time (in jiffies, assuming HZ=100 so this is 100ms)
/// From mt7996.h: #define MT7996_WATCHDOG_TIME (HZ / 10)
pub const MT7996_WATCHDOG_TIME_MS: u32 = 100;

/// Reset timeout (in jiffies, 30 seconds)
/// From mt7996.h: #define MT7996_RESET_TIMEOUT (30 * HZ)
pub const MT7996_RESET_TIMEOUT_MS: u32 = 30_000;

// ============================================================================
// Ring Sizes
// ============================================================================

/// TX ring size for data
/// From mt7996.h: #define MT7996_TX_RING_SIZE 2048
pub const MT7996_TX_RING_SIZE: u32 = 2048;

/// TX MCU ring size
/// From mt7996.h: #define MT7996_TX_MCU_RING_SIZE 256
pub const MT7996_TX_MCU_RING_SIZE: u32 = 256;

/// TX firmware download ring size
/// From mt7996.h: #define MT7996_TX_FWDL_RING_SIZE 128
pub const MT7996_TX_FWDL_RING_SIZE: u32 = 128;

/// RX ring size for data
/// From mt7996.h: #define MT7996_RX_RING_SIZE 1536
pub const MT7996_RX_RING_SIZE: u32 = 1536;

/// RX MCU ring size
/// From mt7996.h: #define MT7996_RX_MCU_RING_SIZE 512
pub const MT7996_RX_MCU_RING_SIZE: u32 = 512;

/// RX MCU ring size for WA
/// From mt7996.h: #define MT7996_RX_MCU_RING_SIZE_WA 1024
pub const MT7996_RX_MCU_RING_SIZE_WA: u32 = 1024;

// ============================================================================
// Device IDs
// ============================================================================

/// MT7996 primary device ID
/// From mt7996.h: #define MT7996_DEVICE_ID 0x7990
pub const MT7996_DEVICE_ID: u16 = 0x7990;

/// MT7996 secondary device ID (HIF2)
/// From mt7996.h: #define MT7996_DEVICE_ID_2 0x7991
pub const MT7996_DEVICE_ID_2: u16 = 0x7991;

/// MT7992 primary device ID
/// From mt7996.h: #define MT7992_DEVICE_ID 0x7992
pub const MT7992_DEVICE_ID: u16 = 0x7992;

/// MT7992 secondary device ID
/// From mt7996.h: #define MT7992_DEVICE_ID_2 0x799a
pub const MT7992_DEVICE_ID_2: u16 = 0x799a;

/// MT7990 primary device ID
/// From mt7996.h: #define MT7990_DEVICE_ID 0x7993
pub const MT7990_DEVICE_ID: u16 = 0x7993;

/// MT7990 secondary device ID
/// From mt7996.h: #define MT7990_DEVICE_ID_2 0x799b
pub const MT7990_DEVICE_ID_2: u16 = 0x799b;

// ============================================================================
// EEPROM Constants
// ============================================================================

/// EEPROM size
/// From mt7996.h: #define MT7996_EEPROM_SIZE 7680
pub const MT7996_EEPROM_SIZE: usize = 7680;

/// EEPROM block size
/// From mt7996.h: #define MT7996_EEPROM_BLOCK_SIZE 16
pub const MT7996_EEPROM_BLOCK_SIZE: usize = 16;

// ============================================================================
// Token Constants
// ============================================================================

/// Token size
/// From mt7996.h: #define MT7996_TOKEN_SIZE 16384
pub const MT7996_TOKEN_SIZE: u32 = 16384;

/// HW token size
/// From mt7996.h: #define MT7996_HW_TOKEN_SIZE 8192
pub const MT7996_HW_TOKEN_SIZE: u32 = 8192;

// ============================================================================
// Rate Constants
// ============================================================================

/// CFEND rate default (OFDM 24M)
/// From mt7996.h: #define MT7996_CFEND_RATE_DEFAULT 0x49
pub const MT7996_CFEND_RATE_DEFAULT: u8 = 0x49;

/// CFEND rate 11B (LP, 11M)
/// From mt7996.h: #define MT7996_CFEND_RATE_11B 0x03
pub const MT7996_CFEND_RATE_11B: u8 = 0x03;

/// MT7992 CFEND rate default (OFDM 6M)
/// From mt7996.h: #define MT7992_CFEND_RATE_DEFAULT 0x4b
pub const MT7992_CFEND_RATE_DEFAULT: u8 = 0x4b;

// ============================================================================
// IBF (Implicit Beamforming) Constants
// ============================================================================

/// IBF maximum NC
pub const MT7996_IBF_MAX_NC: u8 = 2;

/// IBF timeout
pub const MT7996_IBF_TIMEOUT: u8 = 0x18;

/// IBF timeout legacy
pub const MT7996_IBF_TIMEOUT_LEGACY: u8 = 0x48;

/// MT7992 IBF timeout
pub const MT7992_IBF_TIMEOUT: u8 = 0xff;

// ============================================================================
// SKU Constants
// ============================================================================

/// SKU rate number
pub const MT7996_SKU_RATE_NUM: u32 = 417;

/// SKU path number
pub const MT7996_SKU_PATH_NUM: u32 = 494;

// ============================================================================
// TWT Constants
// ============================================================================

/// Maximum TWT agreements
pub const MT7996_MAX_TWT_AGRT: u8 = 16;

/// Maximum STA TWT agreements
pub const MT7996_MAX_STA_TWT_AGRT: u8 = 8;

/// Minimum TWT duration
pub const MT7996_MIN_TWT_DUR: u32 = 64;

// ============================================================================
// Rate Tables
// ============================================================================

/// Basic rates table index
/// From mt7996.h: #define MT7996_BASIC_RATES_TBL 31
pub const MT7996_BASIC_RATES_TBL: u8 = 31;

/// Beacon rates table index
/// From mt7996.h: #define MT7996_BEACON_RATES_TBL 25
pub const MT7996_BEACON_RATES_TBL: u8 = 25;

// ============================================================================
// Thermal Constants
// ============================================================================

/// Thermal throttle maximum
pub const MT7996_THERMAL_THROTTLE_MAX: u8 = 100;

/// CDEV throttle maximum
pub const MT7996_CDEV_THROTTLE_MAX: u8 = 99;

/// Critical temperature index
pub const MT7996_CRIT_TEMP_IDX: u8 = 0;

/// Maximum temperature index
pub const MT7996_MAX_TEMP_IDX: u8 = 1;

/// Critical temperature (Celsius)
pub const MT7996_CRIT_TEMP: u8 = 110;

/// Maximum temperature (Celsius)
pub const MT7996_MAX_TEMP: u8 = 120;

// ============================================================================
// RRO Constants
// ============================================================================

/// Maximum HIF RXD in page
pub const MT7996_MAX_HIF_RXD_IN_PG: usize = 5;

/// RRO MSDU page hash size
pub const MT7996_RRO_MSDU_PG_HASH_SIZE: usize = 127;

/// RRO maximum session
pub const MT7996_RRO_MAX_SESSION: usize = 1024;

/// RRO window maximum length
pub const MT7996_RRO_WINDOW_MAX_LEN: usize = 1024;

/// RRO address element length
pub const MT7996_RRO_ADDR_ELEM_LEN: usize = 128;

/// RRO BA bitmap length
pub const MT7996_RRO_BA_BITMAP_LEN: usize = 2;

/// RRO 3.1 MSDU page CR count
pub const MT7996_RRO_MSDU_PG_CR_CNT: usize = 8;

/// RRO 3.1 MSDU page size per CR
pub const MT7996_RRO_MSDU_PG_SIZE_PER_CR: usize = 0x10000;

/// Maximum RRO RRS rings
pub const MT7996_MAX_RRO_RRS_RING: usize = 4;

// ============================================================================
// TX Queue IDs (MT7996 specific hardware mapping)
// ============================================================================

/// TX queue IDs for MT7996
/// From mt7996.h: enum mt7996_txq_id
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt7996TxqId {
    /// Firmware download queue
    Fwdl = 16,
    /// MCU WM queue
    McuWm = 17,
    /// Band 0 data queue
    Band0 = 18,
    /// Band 1 data queue
    Band1 = 19,
    /// MCU WA queue
    McuWa = 20,
    /// Band 2 data queue
    Band2 = 21,
}

// ============================================================================
// RX Queue IDs (MT7996 specific hardware mapping)
// ============================================================================

/// RX queue IDs for MT7996 family
/// From mt7996.h: enum mt7996_rxq_id
/// Note: Some IDs are reused for different queues depending on chip variant
pub mod rxq_id {
    /// MCU WM RX queue
    pub const MCU_WM: u32 = 0;
    /// MCU WA RX queue
    pub const MCU_WA: u32 = 1;
    /// MCU WA main RX queue
    pub const MCU_WA_MAIN: u32 = 2;
    /// MCU WA ext RX queue (for MT7992)
    pub const MCU_WA_EXT: u32 = 3;
    /// MCU WA tri RX queue (same as MCU_WA_EXT)
    pub const MCU_WA_TRI: u32 = 3;
    /// Band 0 data RX queue
    pub const BAND0: u32 = 4;
    /// Band 1 data RX queue (for MT7992, same as BAND2)
    pub const BAND1: u32 = 5;
    /// Band 2 data RX queue (same as BAND1)
    pub const BAND2: u32 = 5;
    /// RRO Band 0
    pub const RRO_BAND0: u32 = 8;
    /// RRO Band 1 (same as TXFREE0/TXFREE1)
    pub const RRO_BAND1: u32 = 9;
    /// RRO Band 2 (same as MT7990_TXFREE0)
    pub const RRO_BAND2: u32 = 6;
    /// MSDU page Band 0
    pub const MSDU_PG_BAND0: u32 = 10;
    /// MSDU page Band 1
    pub const MSDU_PG_BAND1: u32 = 11;
    /// MSDU page Band 2
    pub const MSDU_PG_BAND2: u32 = 12;
    /// TX free 0 (same as RRO_BAND1)
    pub const TXFREE0: u32 = 9;
    /// TX free 1 (same as RRO_BAND1)
    pub const TXFREE1: u32 = 9;
    /// TX free 2 (same as MT7990_TXFREE1)
    pub const TXFREE2: u32 = 7;
    /// RRO indication (same as MCU_WM)
    pub const RRO_IND: u32 = 0;
    /// RRO RXDMAD C (same as MCU_WM)
    pub const RRO_RXDMAD_C: u32 = 0;
    /// MT7990 TX free 0 (same as RRO_BAND2)
    pub const MT7990_TXFREE0: u32 = 6;
    /// MT7990 TX free 1 (same as TXFREE2)
    pub const MT7990_TXFREE1: u32 = 7;
}

/// Re-export common RX queue IDs for compatibility
pub use rxq_id::*;

// ============================================================================
// RAM Types
// ============================================================================

/// RAM types for firmware
/// From mt7996.h: enum mt7996_ram_type
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt7996RamType {
    /// WiFi Manager
    Wm = 0,
    /// WiFi Agent
    Wa = 1,
    /// DSP
    Dsp = 2,
}

// ============================================================================
// Variant Types
// ============================================================================

/// MT7996 variant types
/// From mt7996.h: enum mt7996_var_type
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt7996VarType {
    /// 4+4+4 antenna configuration
    Type444 = 0,
    /// 2+3+3 antenna configuration
    Type233 = 1,
}

/// MT7992 variant types
/// From mt7996.h: enum mt7992_var_type
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt7992VarType {
    /// 4+4 antenna configuration
    Type44 = 0,
    /// 2+3 antenna configuration
    Type23 = 1,
}

/// MT7990 variant types
/// From mt7996.h: enum mt7990_var_type
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt7990VarType {
    /// 2+3 antenna configuration
    Type23 = 0,
}

/// FEM (Front End Module) types
/// From mt7996.h: enum mt7996_fem_type
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt7996FemType {
    /// External FEM
    Ext = 0,
    /// Internal FEM
    Int = 1,
    /// Mixed FEM
    Mix = 2,
}

// ============================================================================
// WFDMA Types
// ============================================================================

/// WFDMA types
/// From mt7996.h enum (unnamed)
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WfdmaType {
    Wfdma0 = 0x0,
    Wfdma1 = 0x1,
    WfdmaExt = 0x2,
}

/// Maximum WFDMA count
pub const MT_WFDMA_MAX: usize = 3;

// ============================================================================
// RDD (Radar Detection) Types
// ============================================================================

/// RDD index
/// From mt7996.h: enum rdd_idx
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RddIdx {
    /// RDD index for band 2
    Band2 = 0,
    /// RDD index for band 1
    Band1 = 1,
    /// RDD index for background chain
    Background = 2,
}

/// RDD commands
/// From mt7996.h: enum mt7996_rdd_cmd
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mt7996RddCmd {
    Stop = 0,
    Start = 1,
    DetMode = 2,
    RadarEmulate = 3,
    StartTxq = 20,
    CacStart = 50,
    CacEnd = 51,
    NormalStart = 52,
    DisableDfsCal = 53,
    PulseDbg = 54,
    ReadPulse = 55,
    ResumeBf = 56,
    IrqOff = 57,
}

// ============================================================================
// WED RRO Address Fields
// ============================================================================

/// WED RRO address signature mask (bits 31:24)
pub const WED_RRO_ADDR_SIGNATURE_MASK: u32 = genmask(31, 24);

/// WED RRO address count mask (bits 14:4)
pub const WED_RRO_ADDR_COUNT_MASK: u32 = genmask(14, 4);

/// WED RRO address head high mask (bits 3:0)
pub const WED_RRO_ADDR_HEAD_HIGH_MASK: u32 = genmask(3, 0);

// ============================================================================
// RRO HIF Data Fields
// ============================================================================

/// RRO HIF data1 last segment mask (bit 30)
pub const RRO_HIF_DATA1_LS_MASK: u32 = bit(30);

/// RRO HIF data1 SDL mask (bits 29:16)
pub const RRO_HIF_DATA1_SDL_MASK: u32 = genmask(29, 16);

/// RRO HIF data4 RX token ID mask (bits 15:0)
pub const RRO_HIF_DATA4_RX_TOKEN_ID_MASK: u32 = genmask(15, 0);

// ============================================================================
// MSDU Page Info Fields
// ============================================================================

/// MSDU page info owner mask (bit 31)
pub const MSDU_PAGE_INFO_OWNER_MASK: u32 = bit(31);

/// MSDU page info page high mask (bits 3:0)
pub const MSDU_PAGE_INFO_PG_HIGH_MASK: u32 = genmask(3, 0);

// ============================================================================
// Helper Functions
// ============================================================================

/// Check if device ID is MT7996
pub const fn is_mt7996(device_id: u16) -> bool {
    device_id == MT7996_DEVICE_ID || device_id == MT7996_DEVICE_ID_2
}

/// Check if device ID is MT7992
pub const fn is_mt7992(device_id: u16) -> bool {
    device_id == MT7992_DEVICE_ID || device_id == MT7992_DEVICE_ID_2
}

/// Check if device ID is MT7990
pub const fn is_mt7990(device_id: u16) -> bool {
    device_id == MT7990_DEVICE_ID || device_id == MT7990_DEVICE_ID_2
}

/// Get hardware queue ID from TX queue enum
pub const fn txq_hw_id(q: Mt7996TxqId) -> u32 {
    q as u32
}

/// Get hardware queue ID from RX queue constant
/// RX queues are already u32 constants, so this is just a pass-through
pub const fn rxq_hw_id(q: u32) -> u32 {
    q
}
