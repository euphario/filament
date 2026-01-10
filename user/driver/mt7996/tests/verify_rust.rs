//! MT7996 Rust Constant Verification
//!
//! This standalone program prints all critical constants from the Rust translation.
//! The output format matches verify_macros.c so they can be diff'd directly.
//!
//! To compile:
//!   rustc verify_rust.rs -o verify_rust
//!
//! To verify:
//!   ./verify_macros > c_values.txt
//!   ./verify_rust > rust_values.txt
//!   diff c_values.txt rust_values.txt
//!
//! If no differences, the translation is correct.

// ============================================================================
// Helper Functions (matching Linux GENMASK/BIT exactly)
// ============================================================================

const fn bit(n: u32) -> u32 {
    1u32 << n
}

const fn genmask(hi: u32, lo: u32) -> u32 {
    (((1u64 << (hi - lo + 1)) - 1) << lo) as u32
}

// ============================================================================
// From mt76/dma.h
// ============================================================================

const MT_RING_SIZE: u32 = 0x10;

const MT_DMA_CTL_SD_LEN1: u32 = genmask(13, 0);
const MT_DMA_CTL_LAST_SEC1: u32 = bit(14);
const MT_DMA_CTL_BURST: u32 = bit(15);
const MT_DMA_CTL_SD_LEN0: u32 = genmask(29, 16);
const MT_DMA_CTL_LAST_SEC0: u32 = bit(30);
const MT_DMA_CTL_DMA_DONE: u32 = bit(31);
const MT_DMA_CTL_TO_HOST: u32 = bit(8);
const MT_DMA_CTL_TO_HOST_A: u32 = bit(12);
const MT_DMA_CTL_DROP: u32 = bit(14);
const MT_DMA_CTL_TOKEN: u32 = genmask(31, 16);
const MT_DMA_CTL_SDP1_H: u32 = genmask(19, 16);
const MT_DMA_CTL_SDP0_H: u32 = genmask(3, 0);
const MT_DMA_CTL_WO_DROP: u32 = bit(8);

const MT_DMA_PPE_CPU_REASON: u32 = genmask(15, 11);
const MT_DMA_PPE_ENTRY: u32 = genmask(30, 16);
const MT_DMA_INFO_DMA_FRAG: u32 = bit(9);
const MT_DMA_INFO_PPE_VLD: u32 = bit(31);

const MT_DMA_CTL_PN_CHK_FAIL: u32 = bit(13);
const MT_DMA_CTL_VER_MASK: u32 = bit(7);

const MT_DMA_SDP0: u32 = genmask(15, 0);
const MT_DMA_TOKEN_ID: u32 = genmask(31, 16);
const MT_DMA_MAGIC_MASK: u32 = genmask(31, 28);  // CRITICAL: bits 31:28 = 0xF0000000
const MT_DMA_RRO_EN: u32 = bit(13);

const MT_DMA_MAGIC_CNT: u32 = 16;

const MT_DMA_WED_IND_CMD_CNT: u32 = 8;
const MT_DMA_WED_IND_REASON: u32 = genmask(15, 12);

const MT_DMA_HDR_LEN: u32 = 4;
const MT_RX_INFO_LEN: u32 = 4;
const MT_FCE_INFO_LEN: u32 = 4;
const MT_RX_RXWI_LEN: u32 = 32;

// ============================================================================
// From mt7996/regs.h - WFDMA0 Registers
// ============================================================================

const MT_WFDMA0_BASE: u32 = 0xd4000;
const fn mt_wfdma0(ofs: u32) -> u32 { MT_WFDMA0_BASE + ofs }

const MT_WFDMA0_RST: u32 = mt_wfdma0(0x100);
const MT_WFDMA0_RST_LOGIC_RST: u32 = bit(4);
const MT_WFDMA0_RST_DMASHDL_ALL_RST: u32 = bit(5);

const MT_WFDMA0_BUSY_ENA: u32 = mt_wfdma0(0x13c);
const MT_WFDMA0_BUSY_ENA_TX_FIFO0: u32 = bit(0);
const MT_WFDMA0_BUSY_ENA_TX_FIFO1: u32 = bit(1);
const MT_WFDMA0_BUSY_ENA_RX_FIFO: u32 = bit(2);

const MT_WFDMA0_GLO_CFG: u32 = mt_wfdma0(0x208);
const MT_WFDMA0_GLO_CFG_TX_DMA_EN: u32 = bit(0);
const MT_WFDMA0_GLO_CFG_RX_DMA_EN: u32 = bit(2);
const MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2: u32 = bit(21);
const MT_WFDMA0_GLO_CFG_EXT_EN: u32 = bit(26);
const MT_WFDMA0_GLO_CFG_OMIT_RX_INFO: u32 = bit(27);
const MT_WFDMA0_GLO_CFG_OMIT_TX_INFO: u32 = bit(28);

const MT_WFDMA0_RST_DTX_PTR: u32 = mt_wfdma0(0x20c);
const MT_WFDMA0_PRI_DLY_INT_CFG0: u32 = mt_wfdma0(0x2f0);
const MT_WFDMA0_PRI_DLY_INT_CFG1: u32 = mt_wfdma0(0x2f4);
const MT_WFDMA0_PRI_DLY_INT_CFG2: u32 = mt_wfdma0(0x2f8);

// RX pause thresholds
const MT_WFDMA0_PAUSE_RX_Q_45_TH: u32 = mt_wfdma0(0x268);
const MT_WFDMA0_PAUSE_RX_Q_67_TH: u32 = mt_wfdma0(0x26c);
const MT_WFDMA0_PAUSE_RX_Q_89_TH: u32 = mt_wfdma0(0x270);
// Note: 0x274, 0x278 reserved
const MT_WFDMA0_PAUSE_RX_Q_RRO_TH: u32 = mt_wfdma0(0x27c);

// Global config extensions
const MT_WFDMA0_GLO_CFG_EXT0: u32 = mt_wfdma0(0x2b0);
const MT_WFDMA0_GLO_CFG_EXT0_RX_WB_RXD: u32 = bit(18);
const MT_WFDMA0_GLO_CFG_EXT0_WED_MERGE_MODE: u32 = bit(14);

const MT_WFDMA0_GLO_CFG_EXT1: u32 = mt_wfdma0(0x2b4);
const MT_WFDMA0_GLO_CFG_EXT1_CALC_MODE: u32 = bit(31);
const MT_WFDMA0_GLO_CFG_EXT1_TX_FCTRL_MODE: u32 = bit(28);

// Interrupt registers
const MT_INT_SOURCE_CSR: u32 = mt_wfdma0(0x200);
const MT_INT_MASK_CSR: u32 = mt_wfdma0(0x204);

// MCU command
const MT_WFDMA0_MCU_CMD: u32 = mt_wfdma0(0x1f0);
const MT_MCU_CMD_STOP_DMA: u32 = bit(2);
const MT_MCU_CMD_RESET_DONE: u32 = bit(3);
const MT_MCU_CMD_RECOVERY_DONE: u32 = bit(4);
const MT_MCU_CMD_NORMAL_STATE: u32 = bit(5);
const MT_MCU_CMD_WA_WDT: u32 = bit(31);
const MT_MCU_CMD_WM_WDT: u32 = bit(30);

// ============================================================================
// From mt7996/regs.h - WFDMA0 PCIE1 (HIF2)
// ============================================================================

const MT_WFDMA0_PCIE1_BASE: u32 = 0xd8000;
const fn mt_wfdma0_pcie1(ofs: u32) -> u32 { MT_WFDMA0_PCIE1_BASE + ofs }

const MT_HIF1_OFS: u32 = MT_WFDMA0_PCIE1_BASE - MT_WFDMA0_BASE;

// ============================================================================
// From mt7996/regs.h - WFDMA Extended CSR
// ============================================================================

const MT_WFDMA_EXT_CSR_BASE: u32 = 0xd7000;
const fn mt_wfdma_ext_csr(ofs: u32) -> u32 { MT_WFDMA_EXT_CSR_BASE + ofs }

const MT_WFDMA_HOST_CONFIG: u32 = mt_wfdma_ext_csr(0x30);
const MT_WFDMA_HIF_MISC: u32 = mt_wfdma_ext_csr(0x44);
const MT_WFDMA_HIF_MISC_BUSY: u32 = bit(0);

// ============================================================================
// From mt7996/mt7996.h - Device IDs
// ============================================================================

const MT7996_DEVICE_ID: u32 = 0x7990;
const MT7996_DEVICE_ID_2: u32 = 0x7991;
const MT7992_DEVICE_ID: u32 = 0x7992;
const MT7992_DEVICE_ID_2: u32 = 0x799a;
const MT7990_DEVICE_ID: u32 = 0x7993;
const MT7990_DEVICE_ID_2: u32 = 0x799b;

// ============================================================================
// From mt7996/mt7996.h - Ring Sizes
// ============================================================================

const MT7996_TX_RING_SIZE: u32 = 2048;
const MT7996_TX_MCU_RING_SIZE: u32 = 256;
const MT7996_TX_FWDL_RING_SIZE: u32 = 128;
const MT7996_RX_RING_SIZE: u32 = 1536;
const MT7996_RX_MCU_RING_SIZE: u32 = 512;
const MT7996_RX_MCU_RING_SIZE_WA: u32 = 1024;

// ============================================================================
// From mt76_connac3_mac.h - TXD Size
// ============================================================================

const MT_TXD_SIZE: u32 = 8 * 4;

// ============================================================================
// Main - Print all values in same format as C program
// ============================================================================

macro_rules! print_macro {
    ($name:ident) => {
        println!("{:<40} = 0x{:08x} ({})", stringify!($name), $name, $name);
    };
}

fn main() {
    println!("=== MT7996 Macro Verification ===\n");

    println!("=== DMA Definitions (dma.h) ===");
    print_macro!(MT_RING_SIZE);
    print_macro!(MT_DMA_CTL_SD_LEN1);
    print_macro!(MT_DMA_CTL_LAST_SEC1);
    print_macro!(MT_DMA_CTL_BURST);
    print_macro!(MT_DMA_CTL_SD_LEN0);
    print_macro!(MT_DMA_CTL_LAST_SEC0);
    print_macro!(MT_DMA_CTL_DMA_DONE);
    print_macro!(MT_DMA_CTL_TO_HOST);
    print_macro!(MT_DMA_CTL_TO_HOST_A);
    print_macro!(MT_DMA_CTL_DROP);
    print_macro!(MT_DMA_CTL_TOKEN);
    print_macro!(MT_DMA_CTL_SDP1_H);
    print_macro!(MT_DMA_CTL_SDP0_H);
    print_macro!(MT_DMA_CTL_WO_DROP);
    print_macro!(MT_DMA_PPE_CPU_REASON);
    print_macro!(MT_DMA_PPE_ENTRY);
    print_macro!(MT_DMA_INFO_DMA_FRAG);
    print_macro!(MT_DMA_INFO_PPE_VLD);
    print_macro!(MT_DMA_CTL_PN_CHK_FAIL);
    print_macro!(MT_DMA_CTL_VER_MASK);
    print_macro!(MT_DMA_SDP0);
    print_macro!(MT_DMA_TOKEN_ID);
    print_macro!(MT_DMA_MAGIC_MASK);
    print_macro!(MT_DMA_RRO_EN);
    print_macro!(MT_DMA_MAGIC_CNT);
    print_macro!(MT_DMA_WED_IND_CMD_CNT);
    print_macro!(MT_DMA_WED_IND_REASON);
    print_macro!(MT_DMA_HDR_LEN);
    print_macro!(MT_RX_INFO_LEN);
    print_macro!(MT_FCE_INFO_LEN);
    print_macro!(MT_RX_RXWI_LEN);
    println!();

    println!("=== WFDMA0 Registers (regs.h) ===");
    print_macro!(MT_WFDMA0_BASE);
    print_macro!(MT_WFDMA0_RST);
    print_macro!(MT_WFDMA0_RST_LOGIC_RST);
    print_macro!(MT_WFDMA0_RST_DMASHDL_ALL_RST);
    print_macro!(MT_WFDMA0_BUSY_ENA);
    print_macro!(MT_WFDMA0_BUSY_ENA_TX_FIFO0);
    print_macro!(MT_WFDMA0_BUSY_ENA_TX_FIFO1);
    print_macro!(MT_WFDMA0_BUSY_ENA_RX_FIFO);
    print_macro!(MT_WFDMA0_GLO_CFG);
    print_macro!(MT_WFDMA0_GLO_CFG_TX_DMA_EN);
    print_macro!(MT_WFDMA0_GLO_CFG_RX_DMA_EN);
    print_macro!(MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2);
    print_macro!(MT_WFDMA0_GLO_CFG_EXT_EN);
    print_macro!(MT_WFDMA0_GLO_CFG_OMIT_RX_INFO);
    print_macro!(MT_WFDMA0_GLO_CFG_OMIT_TX_INFO);
    print_macro!(MT_WFDMA0_RST_DTX_PTR);
    print_macro!(MT_WFDMA0_PRI_DLY_INT_CFG0);
    print_macro!(MT_WFDMA0_PRI_DLY_INT_CFG1);
    print_macro!(MT_WFDMA0_PRI_DLY_INT_CFG2);
    print_macro!(MT_WFDMA0_PAUSE_RX_Q_45_TH);
    print_macro!(MT_WFDMA0_PAUSE_RX_Q_67_TH);
    print_macro!(MT_WFDMA0_PAUSE_RX_Q_89_TH);
    print_macro!(MT_WFDMA0_PAUSE_RX_Q_RRO_TH);
    print_macro!(MT_WFDMA0_GLO_CFG_EXT0);
    print_macro!(MT_WFDMA0_GLO_CFG_EXT0_RX_WB_RXD);
    print_macro!(MT_WFDMA0_GLO_CFG_EXT0_WED_MERGE_MODE);
    print_macro!(MT_WFDMA0_GLO_CFG_EXT1);
    print_macro!(MT_WFDMA0_GLO_CFG_EXT1_CALC_MODE);
    print_macro!(MT_WFDMA0_GLO_CFG_EXT1_TX_FCTRL_MODE);
    print_macro!(MT_INT_SOURCE_CSR);
    print_macro!(MT_INT_MASK_CSR);
    print_macro!(MT_WFDMA0_MCU_CMD);
    print_macro!(MT_MCU_CMD_STOP_DMA);
    print_macro!(MT_MCU_CMD_RESET_DONE);
    print_macro!(MT_MCU_CMD_RECOVERY_DONE);
    print_macro!(MT_MCU_CMD_NORMAL_STATE);
    print_macro!(MT_MCU_CMD_WA_WDT);
    print_macro!(MT_MCU_CMD_WM_WDT);
    println!();

    println!("=== WFDMA0 PCIE1 (HIF2) ===");
    print_macro!(MT_WFDMA0_PCIE1_BASE);
    print_macro!(MT_HIF1_OFS);
    println!();

    println!("=== WFDMA Extended CSR ===");
    print_macro!(MT_WFDMA_EXT_CSR_BASE);
    print_macro!(MT_WFDMA_HOST_CONFIG);
    print_macro!(MT_WFDMA_HIF_MISC);
    print_macro!(MT_WFDMA_HIF_MISC_BUSY);
    println!();

    println!("=== Device IDs (mt7996.h) ===");
    print_macro!(MT7996_DEVICE_ID);
    print_macro!(MT7996_DEVICE_ID_2);
    print_macro!(MT7992_DEVICE_ID);
    print_macro!(MT7992_DEVICE_ID_2);
    print_macro!(MT7990_DEVICE_ID);
    print_macro!(MT7990_DEVICE_ID_2);
    println!();

    println!("=== Ring Sizes (mt7996.h) ===");
    print_macro!(MT7996_TX_RING_SIZE);
    print_macro!(MT7996_TX_MCU_RING_SIZE);
    print_macro!(MT7996_TX_FWDL_RING_SIZE);
    print_macro!(MT7996_RX_RING_SIZE);
    print_macro!(MT7996_RX_MCU_RING_SIZE);
    print_macro!(MT7996_RX_MCU_RING_SIZE_WA);
    println!();

    println!("=== TXD Size (connac3_mac.h) ===");
    print_macro!(MT_TXD_SIZE);
    println!();

    println!("=== CRITICAL: MAGIC_MASK Location ===");
    println!("MT_DMA_MAGIC_MASK = 0x{:08x}", MT_DMA_MAGIC_MASK);
    println!("This MUST be bits 31:28 (0xF0000000), NOT bits 15:12!");
    if MT_DMA_MAGIC_MASK == 0xF0000000 {
        println!("PASS: MAGIC_MASK is correct (0xF0000000)");
    } else {
        println!("FAIL: MAGIC_MASK is WRONG! Expected 0xF0000000, got 0x{:08x}", MT_DMA_MAGIC_MASK);
    }
    println!();

    println!("=== Verification Complete ===");
}
