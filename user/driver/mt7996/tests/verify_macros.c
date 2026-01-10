/*
 * MT7996 Macro Verification Test Program
 *
 * This program prints the values of all critical macros from the Linux
 * mt7996/mt76 driver headers. The output can be compared against the
 * Rust translations to verify correctness.
 *
 * To compile (needs Linux kernel headers):
 *   cd /path/to/linux
 *   gcc -I drivers/net/wireless/mediatek/mt76 \
 *       -I drivers/net/wireless/mediatek/mt76/mt7996 \
 *       -include linux/types.h \
 *       -DCONFIG_NET_MEDIATEK_SOC_WED=0 \
 *       -o verify_macros verify_macros.c
 *
 * Or use the standalone version that defines its own GENMASK/BIT:
 *   gcc -o verify_macros verify_macros.c
 */

#include <stdio.h>
#include <stdint.h>

/* Standalone GENMASK and BIT definitions */
#define BIT(n) (1UL << (n))
#define GENMASK(hi, lo) (((1ULL << ((hi) - (lo) + 1)) - 1) << (lo))

/* ============================================================================
 * From mt76/dma.h
 * ============================================================================ */

#define MT_RING_SIZE                    0x10

#define MT_DMA_CTL_SD_LEN1              GENMASK(13, 0)
#define MT_DMA_CTL_LAST_SEC1            BIT(14)
#define MT_DMA_CTL_BURST                BIT(15)
#define MT_DMA_CTL_SD_LEN0              GENMASK(29, 16)
#define MT_DMA_CTL_LAST_SEC0            BIT(30)
#define MT_DMA_CTL_DMA_DONE             BIT(31)
#define MT_DMA_CTL_TO_HOST              BIT(8)
#define MT_DMA_CTL_TO_HOST_A            BIT(12)
#define MT_DMA_CTL_DROP                 BIT(14)
#define MT_DMA_CTL_TOKEN                GENMASK(31, 16)
#define MT_DMA_CTL_SDP1_H               GENMASK(19, 16)
#define MT_DMA_CTL_SDP0_H               GENMASK(3, 0)
#define MT_DMA_CTL_WO_DROP              BIT(8)

#define MT_DMA_PPE_CPU_REASON           GENMASK(15, 11)
#define MT_DMA_PPE_ENTRY                GENMASK(30, 16)
#define MT_DMA_INFO_DMA_FRAG            BIT(9)
#define MT_DMA_INFO_PPE_VLD             BIT(31)

#define MT_DMA_CTL_PN_CHK_FAIL          BIT(13)
#define MT_DMA_CTL_VER_MASK             BIT(7)

#define MT_DMA_SDP0                     GENMASK(15, 0)
#define MT_DMA_TOKEN_ID                 GENMASK(31, 16)
#define MT_DMA_MAGIC_MASK               GENMASK(31, 28)
#define MT_DMA_RRO_EN                   BIT(13)

#define MT_DMA_MAGIC_CNT                16

#define MT_DMA_WED_IND_CMD_CNT          8
#define MT_DMA_WED_IND_REASON           GENMASK(15, 12)

#define MT_DMA_HDR_LEN                  4
#define MT_RX_INFO_LEN                  4
#define MT_FCE_INFO_LEN                 4
#define MT_RX_RXWI_LEN                  32

/* ============================================================================
 * From mt7996/regs.h - WFDMA0 Registers
 * ============================================================================ */

#define MT_WFDMA0_BASE                  0xd4000
#define MT_WFDMA0(ofs)                  (MT_WFDMA0_BASE + (ofs))

#define MT_WFDMA0_RST                   MT_WFDMA0(0x100)
#define MT_WFDMA0_RST_LOGIC_RST         BIT(4)
#define MT_WFDMA0_RST_DMASHDL_ALL_RST   BIT(5)

#define MT_WFDMA0_BUSY_ENA              MT_WFDMA0(0x13c)
#define MT_WFDMA0_BUSY_ENA_TX_FIFO0     BIT(0)
#define MT_WFDMA0_BUSY_ENA_TX_FIFO1     BIT(1)
#define MT_WFDMA0_BUSY_ENA_RX_FIFO      BIT(2)

#define MT_WFDMA0_GLO_CFG               MT_WFDMA0(0x208)
#define MT_WFDMA0_GLO_CFG_TX_DMA_EN     BIT(0)
#define MT_WFDMA0_GLO_CFG_RX_DMA_EN     BIT(2)
#define MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2 BIT(21)
#define MT_WFDMA0_GLO_CFG_EXT_EN        BIT(26)
#define MT_WFDMA0_GLO_CFG_OMIT_RX_INFO  BIT(27)
#define MT_WFDMA0_GLO_CFG_OMIT_TX_INFO  BIT(28)

#define MT_WFDMA0_RST_DTX_PTR           MT_WFDMA0(0x20c)
#define MT_WFDMA0_PRI_DLY_INT_CFG0      MT_WFDMA0(0x2f0)
#define MT_WFDMA0_PRI_DLY_INT_CFG1      MT_WFDMA0(0x2f4)
#define MT_WFDMA0_PRI_DLY_INT_CFG2      MT_WFDMA0(0x2f8)

/* RX pause thresholds */
#define MT_WFDMA0_PAUSE_RX_Q_45_TH      MT_WFDMA0(0x268)
#define MT_WFDMA0_PAUSE_RX_Q_67_TH      MT_WFDMA0(0x26c)
#define MT_WFDMA0_PAUSE_RX_Q_89_TH      MT_WFDMA0(0x270)
/* Note: 0x274, 0x278 reserved - NOT sequential! */
#define MT_WFDMA0_PAUSE_RX_Q_RRO_TH     MT_WFDMA0(0x27c)

/* Global config extensions */
#define MT_WFDMA0_GLO_CFG_EXT0          MT_WFDMA0(0x2b0)
#define MT_WFDMA0_GLO_CFG_EXT0_RX_WB_RXD BIT(18)
#define MT_WFDMA0_GLO_CFG_EXT0_WED_MERGE_MODE BIT(14)

#define MT_WFDMA0_GLO_CFG_EXT1          MT_WFDMA0(0x2b4)
#define MT_WFDMA0_GLO_CFG_EXT1_CALC_MODE BIT(31)
#define MT_WFDMA0_GLO_CFG_EXT1_TX_FCTRL_MODE BIT(28)

/* Interrupt registers */
#define MT_INT_SOURCE_CSR               MT_WFDMA0(0x200)
#define MT_INT_MASK_CSR                 MT_WFDMA0(0x204)

/* MCU command */
#define MT_WFDMA0_MCU_CMD               MT_WFDMA0(0x1f0)
#define MT_MCU_CMD_STOP_DMA             BIT(2)
#define MT_MCU_CMD_RESET_DONE           BIT(3)
#define MT_MCU_CMD_RECOVERY_DONE        BIT(4)
#define MT_MCU_CMD_NORMAL_STATE         BIT(5)
#define MT_MCU_CMD_WA_WDT               BIT(31)
#define MT_MCU_CMD_WM_WDT               BIT(30)

/* ============================================================================
 * From mt7996/regs.h - WFDMA0 PCIE1 (HIF2)
 * ============================================================================ */

#define MT_WFDMA0_PCIE1_BASE            0xd8000
#define MT_WFDMA0_PCIE1(ofs)            (MT_WFDMA0_PCIE1_BASE + (ofs))

#define MT_HIF1_OFS                     (MT_WFDMA0_PCIE1_BASE - MT_WFDMA0_BASE)

/* ============================================================================
 * From mt7996/regs.h - WFDMA Extended CSR
 * ============================================================================ */

#define MT_WFDMA_EXT_CSR_BASE           0xd7000
#define MT_WFDMA_EXT_CSR(ofs)           (MT_WFDMA_EXT_CSR_BASE + (ofs))

#define MT_WFDMA_HOST_CONFIG            MT_WFDMA_EXT_CSR(0x30)
#define MT_WFDMA_HIF_MISC               MT_WFDMA_EXT_CSR(0x44)
#define MT_WFDMA_HIF_MISC_BUSY          BIT(0)

/* ============================================================================
 * From mt7996/mt7996.h - Device IDs
 * ============================================================================ */

#define MT7996_DEVICE_ID                0x7990
#define MT7996_DEVICE_ID_2              0x7991
#define MT7992_DEVICE_ID                0x7992
#define MT7992_DEVICE_ID_2              0x799a
#define MT7990_DEVICE_ID                0x7993
#define MT7990_DEVICE_ID_2              0x799b

/* ============================================================================
 * From mt7996/mt7996.h - Ring Sizes
 * ============================================================================ */

#define MT7996_TX_RING_SIZE             2048
#define MT7996_TX_MCU_RING_SIZE         256
#define MT7996_TX_FWDL_RING_SIZE        128
#define MT7996_RX_RING_SIZE             1536
#define MT7996_RX_MCU_RING_SIZE         512
#define MT7996_RX_MCU_RING_SIZE_WA      1024

/* ============================================================================
 * From mt76_connac3_mac.h - TXD Size
 * ============================================================================ */

#define MT_TXD_SIZE                     (8 * 4)

/* Print macro helper */
#define PRINT_MACRO(name) printf("%-40s = 0x%08lx (%lu)\n", #name, (unsigned long)(name), (unsigned long)(name))

int main(void) {
    printf("=== MT7996 Macro Verification ===\n\n");

    printf("=== DMA Definitions (dma.h) ===\n");
    PRINT_MACRO(MT_RING_SIZE);
    PRINT_MACRO(MT_DMA_CTL_SD_LEN1);
    PRINT_MACRO(MT_DMA_CTL_LAST_SEC1);
    PRINT_MACRO(MT_DMA_CTL_BURST);
    PRINT_MACRO(MT_DMA_CTL_SD_LEN0);
    PRINT_MACRO(MT_DMA_CTL_LAST_SEC0);
    PRINT_MACRO(MT_DMA_CTL_DMA_DONE);
    PRINT_MACRO(MT_DMA_CTL_TO_HOST);
    PRINT_MACRO(MT_DMA_CTL_TO_HOST_A);
    PRINT_MACRO(MT_DMA_CTL_DROP);
    PRINT_MACRO(MT_DMA_CTL_TOKEN);
    PRINT_MACRO(MT_DMA_CTL_SDP1_H);
    PRINT_MACRO(MT_DMA_CTL_SDP0_H);
    PRINT_MACRO(MT_DMA_CTL_WO_DROP);
    PRINT_MACRO(MT_DMA_PPE_CPU_REASON);
    PRINT_MACRO(MT_DMA_PPE_ENTRY);
    PRINT_MACRO(MT_DMA_INFO_DMA_FRAG);
    PRINT_MACRO(MT_DMA_INFO_PPE_VLD);
    PRINT_MACRO(MT_DMA_CTL_PN_CHK_FAIL);
    PRINT_MACRO(MT_DMA_CTL_VER_MASK);
    PRINT_MACRO(MT_DMA_SDP0);
    PRINT_MACRO(MT_DMA_TOKEN_ID);
    PRINT_MACRO(MT_DMA_MAGIC_MASK);
    PRINT_MACRO(MT_DMA_RRO_EN);
    PRINT_MACRO(MT_DMA_MAGIC_CNT);
    PRINT_MACRO(MT_DMA_WED_IND_CMD_CNT);
    PRINT_MACRO(MT_DMA_WED_IND_REASON);
    PRINT_MACRO(MT_DMA_HDR_LEN);
    PRINT_MACRO(MT_RX_INFO_LEN);
    PRINT_MACRO(MT_FCE_INFO_LEN);
    PRINT_MACRO(MT_RX_RXWI_LEN);
    printf("\n");

    printf("=== WFDMA0 Registers (regs.h) ===\n");
    PRINT_MACRO(MT_WFDMA0_BASE);
    PRINT_MACRO(MT_WFDMA0_RST);
    PRINT_MACRO(MT_WFDMA0_RST_LOGIC_RST);
    PRINT_MACRO(MT_WFDMA0_RST_DMASHDL_ALL_RST);
    PRINT_MACRO(MT_WFDMA0_BUSY_ENA);
    PRINT_MACRO(MT_WFDMA0_BUSY_ENA_TX_FIFO0);
    PRINT_MACRO(MT_WFDMA0_BUSY_ENA_TX_FIFO1);
    PRINT_MACRO(MT_WFDMA0_BUSY_ENA_RX_FIFO);
    PRINT_MACRO(MT_WFDMA0_GLO_CFG);
    PRINT_MACRO(MT_WFDMA0_GLO_CFG_TX_DMA_EN);
    PRINT_MACRO(MT_WFDMA0_GLO_CFG_RX_DMA_EN);
    PRINT_MACRO(MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2);
    PRINT_MACRO(MT_WFDMA0_GLO_CFG_EXT_EN);
    PRINT_MACRO(MT_WFDMA0_GLO_CFG_OMIT_RX_INFO);
    PRINT_MACRO(MT_WFDMA0_GLO_CFG_OMIT_TX_INFO);
    PRINT_MACRO(MT_WFDMA0_RST_DTX_PTR);
    PRINT_MACRO(MT_WFDMA0_PRI_DLY_INT_CFG0);
    PRINT_MACRO(MT_WFDMA0_PRI_DLY_INT_CFG1);
    PRINT_MACRO(MT_WFDMA0_PRI_DLY_INT_CFG2);
    PRINT_MACRO(MT_WFDMA0_PAUSE_RX_Q_45_TH);
    PRINT_MACRO(MT_WFDMA0_PAUSE_RX_Q_67_TH);
    PRINT_MACRO(MT_WFDMA0_PAUSE_RX_Q_89_TH);
    PRINT_MACRO(MT_WFDMA0_PAUSE_RX_Q_RRO_TH);
    PRINT_MACRO(MT_WFDMA0_GLO_CFG_EXT0);
    PRINT_MACRO(MT_WFDMA0_GLO_CFG_EXT0_RX_WB_RXD);
    PRINT_MACRO(MT_WFDMA0_GLO_CFG_EXT0_WED_MERGE_MODE);
    PRINT_MACRO(MT_WFDMA0_GLO_CFG_EXT1);
    PRINT_MACRO(MT_WFDMA0_GLO_CFG_EXT1_CALC_MODE);
    PRINT_MACRO(MT_WFDMA0_GLO_CFG_EXT1_TX_FCTRL_MODE);
    PRINT_MACRO(MT_INT_SOURCE_CSR);
    PRINT_MACRO(MT_INT_MASK_CSR);
    PRINT_MACRO(MT_WFDMA0_MCU_CMD);
    PRINT_MACRO(MT_MCU_CMD_STOP_DMA);
    PRINT_MACRO(MT_MCU_CMD_RESET_DONE);
    PRINT_MACRO(MT_MCU_CMD_RECOVERY_DONE);
    PRINT_MACRO(MT_MCU_CMD_NORMAL_STATE);
    PRINT_MACRO(MT_MCU_CMD_WA_WDT);
    PRINT_MACRO(MT_MCU_CMD_WM_WDT);
    printf("\n");

    printf("=== WFDMA0 PCIE1 (HIF2) ===\n");
    PRINT_MACRO(MT_WFDMA0_PCIE1_BASE);
    PRINT_MACRO(MT_HIF1_OFS);
    printf("\n");

    printf("=== WFDMA Extended CSR ===\n");
    PRINT_MACRO(MT_WFDMA_EXT_CSR_BASE);
    PRINT_MACRO(MT_WFDMA_HOST_CONFIG);
    PRINT_MACRO(MT_WFDMA_HIF_MISC);
    PRINT_MACRO(MT_WFDMA_HIF_MISC_BUSY);
    printf("\n");

    printf("=== Device IDs (mt7996.h) ===\n");
    PRINT_MACRO(MT7996_DEVICE_ID);
    PRINT_MACRO(MT7996_DEVICE_ID_2);
    PRINT_MACRO(MT7992_DEVICE_ID);
    PRINT_MACRO(MT7992_DEVICE_ID_2);
    PRINT_MACRO(MT7990_DEVICE_ID);
    PRINT_MACRO(MT7990_DEVICE_ID_2);
    printf("\n");

    printf("=== Ring Sizes (mt7996.h) ===\n");
    PRINT_MACRO(MT7996_TX_RING_SIZE);
    PRINT_MACRO(MT7996_TX_MCU_RING_SIZE);
    PRINT_MACRO(MT7996_TX_FWDL_RING_SIZE);
    PRINT_MACRO(MT7996_RX_RING_SIZE);
    PRINT_MACRO(MT7996_RX_MCU_RING_SIZE);
    PRINT_MACRO(MT7996_RX_MCU_RING_SIZE_WA);
    printf("\n");

    printf("=== TXD Size (connac3_mac.h) ===\n");
    PRINT_MACRO(MT_TXD_SIZE);
    printf("\n");

    printf("=== CRITICAL: MAGIC_MASK Location ===\n");
    printf("MT_DMA_MAGIC_MASK = 0x%08lx\n", (unsigned long)MT_DMA_MAGIC_MASK);
    printf("This MUST be bits 31:28 (0xF0000000), NOT bits 15:12!\n");
    printf("If your Rust code has 0x0000F000, it's WRONG.\n");
    printf("\n");

    printf("=== Verification Complete ===\n");
    return 0;
}
