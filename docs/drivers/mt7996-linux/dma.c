// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * Copyright (C) 2022 MediaTek Inc.
 * Source: https://github.com/openwrt/mt76/blob/master/mt7996/dma.c
 */

#include "mt7996.h"
#include "../dma.h"
#include "mac.h"

int mt7996_init_tx_queues(struct mt7996_phy *phy, int idx, int n_desc,
			  int ring_base, struct mtk_wed_device *wed)
{
	struct mt7996_dev *dev = phy->dev;
	u32 flags = 0;

	if (mtk_wed_device_active(wed)) {
		ring_base += MT_TXQ_ID(0) * MT_RING_SIZE;
		idx -= MT_TXQ_ID(0);

		if (wed == &dev->mt76.mmio.wed_hif2)
			flags = MT_WED_Q_TX(0);
		else
			flags = MT_WED_Q_TX(idx);
	}

	if (mt76_npu_device_active(&dev->mt76))
		flags = MT_NPU_Q_TX(phy->mt76->band_idx);

	return mt76_connac_init_tx_queues(phy->mt76, idx, n_desc,
					  ring_base, wed, flags);
}

static int mt7996_poll_tx(struct napi_struct *napi, int budget)
{
	struct mt7996_dev *dev;

	dev = container_of(napi, struct mt7996_dev, mt76.tx_napi);

	mt76_connac_tx_cleanup(&dev->mt76);
	if (napi_complete_done(napi, 0))
		mt7996_irq_enable(dev, MT_INT_TX_DONE_MCU);

	return 0;
}

static void mt7996_dma_config(struct mt7996_dev *dev)
{
#define Q_CONFIG(q, wfdma, int, id) do {		\
	if (wfdma)					\
		dev->q_wfdma_mask |= (1 << (q));	\
	dev->q_int_mask[(q)] = int;			\
	dev->q_id[(q)] = id;				\
} while (0)

#define MCUQ_CONFIG(q, wfdma, int, id)	Q_CONFIG(q, (wfdma), (int), (id))
#define RXQ_CONFIG(q, wfdma, int, id)	Q_CONFIG(__RXQ(q), (wfdma), (int), (id))
#define TXQ_CONFIG(q, wfdma, int, id)	Q_CONFIG(__TXQ(q), (wfdma), (int), (id))

	/* rx queue */
	RXQ_CONFIG(MT_RXQ_MCU, WFDMA0, MT_INT_RX_DONE_WM, MT7996_RXQ_MCU_WM);
	RXQ_CONFIG(MT_RXQ_MCU_WA, WFDMA0, MT_INT_RX_DONE_WA, MT7996_RXQ_MCU_WA);
	RXQ_CONFIG(MT_RXQ_MAIN, WFDMA0, MT_INT_RX_DONE_BAND0, MT7996_RXQ_BAND0);
	if (mt7996_has_wa(dev))
		RXQ_CONFIG(MT_RXQ_MAIN_WA, WFDMA0, MT_INT_RX_DONE_WA_MAIN,
			   MT7996_RXQ_MCU_WA_MAIN);

	/* ... (rest of queue config) ... */

	/* mcu tx queue */
	MCUQ_CONFIG(MT_MCUQ_FWDL, WFDMA0, MT_INT_TX_DONE_FWDL, MT7996_TXQ_FWDL);
	MCUQ_CONFIG(MT_MCUQ_WM, WFDMA0, MT_INT_TX_DONE_MCU_WM, MT7996_TXQ_MCU_WM);
	if (mt7996_has_wa(dev))
		MCUQ_CONFIG(MT_MCUQ_WA, WFDMA0, MT_INT_TX_DONE_MCU_WA,
			    MT7996_TXQ_MCU_WA);
}

static u32 __mt7996_dma_prefetch_base(u16 *base, u8 depth)
{
	u32 ret = *base << 16 | depth;

	*base = *base + (depth << 4);

	return ret;
}

static void __mt7996_dma_prefetch(struct mt7996_dev *dev, u32 ofs)
{
	u16 base = 0;
	u8 queue, val;

#define PREFETCH(_depth)	(__mt7996_dma_prefetch_base(&base, (_depth)))
	/* prefetch SRAM wrapping boundary for tx/rx ring. */
	/* Tx Command Rings */
	val = is_mt7996(&dev->mt76) ? 2 : 4;
	mt76_wr(dev, MT_MCUQ_EXT_CTRL(MT_MCUQ_FWDL) + ofs, PREFETCH(val));
	mt76_wr(dev, MT_MCUQ_EXT_CTRL(MT_MCUQ_WM) + ofs, PREFETCH(val));
	if (mt7996_has_wa(dev))
		mt76_wr(dev, MT_MCUQ_EXT_CTRL(MT_MCUQ_WA) + ofs, PREFETCH(val));

	/* ... (rest of prefetch) ... */

	mt76_set(dev, WF_WFDMA0_GLO_CFG_EXT1 + ofs, WF_WFDMA0_GLO_CFG_EXT1_CALC_MODE);
#undef PREFETCH
}

void mt7996_dma_prefetch(struct mt7996_dev *dev)
{
	__mt7996_dma_prefetch(dev, 0);
	if (dev->hif2)
		__mt7996_dma_prefetch(dev, MT_WFDMA0_PCIE1(0) - MT_WFDMA0(0));
}

/*
 * mt7996_dma_disable - Disable DMA and optionally reset
 *
 * CRITICAL: When reset=true, this function:
 * 1. Clears RST bits first (no-op if already 0)
 * 2. Sets RST bits (assert reset)
 * 3. Clears GLO_CFG TX/RX enable bits
 *
 * Note: RST is NOT cleared at the end of this function!
 * It stays asserted (0x30) until mt7996_dma_start clears it.
 */
static void mt7996_dma_disable(struct mt7996_dev *dev, bool reset)
{
	u32 hif1_ofs = 0;

	if (dev->hif2)
		hif1_ofs = MT_WFDMA0_PCIE1(0) - MT_WFDMA0(0);

	if (reset) {
		/* Clear first, then set - this is a reset pulse */
		mt76_clear(dev, MT_WFDMA0_RST,
			   MT_WFDMA0_RST_DMASHDL_ALL_RST |
			   MT_WFDMA0_RST_LOGIC_RST);

		mt76_set(dev, MT_WFDMA0_RST,
			 MT_WFDMA0_RST_DMASHDL_ALL_RST |
			 MT_WFDMA0_RST_LOGIC_RST);

		if (dev->hif2) {
			mt76_clear(dev, MT_WFDMA0_RST + hif1_ofs,
				   MT_WFDMA0_RST_DMASHDL_ALL_RST |
				   MT_WFDMA0_RST_LOGIC_RST);

			mt76_set(dev, MT_WFDMA0_RST + hif1_ofs,
				 MT_WFDMA0_RST_DMASHDL_ALL_RST |
				 MT_WFDMA0_RST_LOGIC_RST);
		}
	}

	/* disable DMA */
	mt76_clear(dev, MT_WFDMA0_GLO_CFG,
		   MT_WFDMA0_GLO_CFG_TX_DMA_EN |
		   MT_WFDMA0_GLO_CFG_RX_DMA_EN |
		   MT_WFDMA0_GLO_CFG_OMIT_TX_INFO |
		   MT_WFDMA0_GLO_CFG_OMIT_RX_INFO |
		   MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2);

	if (dev->hif2) {
		mt76_clear(dev, MT_WFDMA0_GLO_CFG + hif1_ofs,
			   MT_WFDMA0_GLO_CFG_TX_DMA_EN |
			   MT_WFDMA0_GLO_CFG_RX_DMA_EN |
			   MT_WFDMA0_GLO_CFG_OMIT_TX_INFO |
			   MT_WFDMA0_GLO_CFG_OMIT_RX_INFO |
			   MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2);
	}
}

/*
 * mt7996_dma_start - Start DMA engines and enable interrupts
 *
 * This is called AFTER rings are programmed.
 * It enables GLO_CFG TX/RX and sets up interrupts.
 */
void mt7996_dma_start(struct mt7996_dev *dev, bool reset, bool wed_reset)
{
	struct mtk_wed_device *wed = &dev->mt76.mmio.wed;
	u32 hif1_ofs = 0;
	u32 irq_mask;

	if (dev->hif2)
		hif1_ofs = MT_WFDMA0_PCIE1(0) - MT_WFDMA0(0);

	/* enable WFDMA Tx/Rx */
	if (!reset) {
		if (mtk_wed_device_active(wed) && mtk_wed_get_rx_capa(wed))
			mt76_set(dev, MT_WFDMA0_GLO_CFG,
				 MT_WFDMA0_GLO_CFG_TX_DMA_EN |
				 MT_WFDMA0_GLO_CFG_OMIT_TX_INFO |
				 MT_WFDMA0_GLO_CFG_EXT_EN);
		else
			mt76_set(dev, MT_WFDMA0_GLO_CFG,
				 MT_WFDMA0_GLO_CFG_TX_DMA_EN |
				 MT_WFDMA0_GLO_CFG_RX_DMA_EN |
				 MT_WFDMA0_GLO_CFG_OMIT_TX_INFO |
				 MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2 |
				 MT_WFDMA0_GLO_CFG_EXT_EN);

		if (dev->hif2)
			mt76_set(dev, MT_WFDMA0_GLO_CFG + hif1_ofs,
				 MT_WFDMA0_GLO_CFG_TX_DMA_EN |
				 MT_WFDMA0_GLO_CFG_RX_DMA_EN |
				 MT_WFDMA0_GLO_CFG_OMIT_TX_INFO |
				 MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2 |
				 MT_WFDMA0_GLO_CFG_EXT_EN);
	}

	/* enable interrupts for TX/RX rings */
	irq_mask = MT_INT_MCU_CMD | MT_INT_RX_DONE_MCU | MT_INT_TX_DONE_MCU;

	if (mt7996_band_valid(dev, MT_BAND0))
		irq_mask |= MT_INT_BAND0_RX_DONE;

	if (mt7996_band_valid(dev, MT_BAND1))
		irq_mask |= MT_INT_BAND1_RX_DONE;

	if (mt7996_band_valid(dev, MT_BAND2))
		irq_mask |= MT_INT_BAND2_RX_DONE | MT_INT_TX_RX_DONE_EXT;

	/* ... (WED handling) ... */

	irq_mask = reset ? MT_INT_MCU_CMD : irq_mask;

	mt7996_irq_enable(dev, irq_mask);
	mt7996_irq_disable(dev, 0);
}

/*
 * mt7996_dma_enable - Configure and enable DMA
 *
 * This is called AFTER rings are allocated but BEFORE they are used.
 * It configures prefetch, thresholds, and calls mt7996_dma_start.
 */
static void mt7996_dma_enable(struct mt7996_dev *dev, bool reset)
{
	u32 hif1_ofs = 0;

	if (dev->hif2)
		hif1_ofs = MT_WFDMA0_PCIE1(0) - MT_WFDMA0(0);

	/* reset dma idx */
	mt76_wr(dev, MT_WFDMA0_RST_DTX_PTR, ~0);
	if (dev->hif2)
		mt76_wr(dev, MT_WFDMA0_RST_DTX_PTR + hif1_ofs, ~0);

	/* configure delay interrupt off */
	mt76_wr(dev, MT_WFDMA0_PRI_DLY_INT_CFG0, 0);
	mt76_wr(dev, MT_WFDMA0_PRI_DLY_INT_CFG1, 0);
	mt76_wr(dev, MT_WFDMA0_PRI_DLY_INT_CFG2, 0);

	/* configure prefetch settings */
	mt7996_dma_prefetch(dev);

	/* hif wait WFDMA idle */
	mt76_set(dev, MT_WFDMA0_BUSY_ENA,
		 MT_WFDMA0_BUSY_ENA_TX_FIFO0 |
		 MT_WFDMA0_BUSY_ENA_TX_FIFO1 |
		 MT_WFDMA0_BUSY_ENA_RX_FIFO);

	mt76_poll(dev, MT_WFDMA_EXT_CSR_HIF_MISC,
		  MT_WFDMA_EXT_CSR_HIF_MISC_BUSY, 0, 1000);

	/* GLO_CFG_EXT0 */
	mt76_set(dev, WF_WFDMA0_GLO_CFG_EXT0,
		 WF_WFDMA0_GLO_CFG_EXT0_RX_WB_RXD |
		 WF_WFDMA0_GLO_CFG_EXT0_WED_MERGE_MODE);

	/* GLO_CFG_EXT1 */
	mt76_set(dev, WF_WFDMA0_GLO_CFG_EXT1,
		 WF_WFDMA0_GLO_CFG_EXT1_TX_FCTRL_MODE);

	/* WFDMA rx threshold */
	mt76_wr(dev, MT_WFDMA0_PAUSE_RX_Q_45_TH, 0xc000c);
	mt76_wr(dev, MT_WFDMA0_PAUSE_RX_Q_67_TH, 0x10008);
	mt76_wr(dev, MT_WFDMA0_PAUSE_RX_Q_89_TH, 0x10008);
	mt76_wr(dev, MT_WFDMA0_PAUSE_RX_Q_RRO_TH, 0x20);

	/* ... (hif2 handling) ... */

	mt7996_dma_start(dev, reset, true);
}

/*
 * mt7996_dma_init - Main DMA initialization
 *
 * Call sequence:
 * 1. mt7996_dma_config() - Set up queue mappings
 * 2. mt76_dma_attach() - Attach DMA ops
 * 3. mt7996_dma_disable(reset=true) - Reset pulse (RST stays asserted!)
 * 4. Allocate all TX/RX queues - Ring programming happens here
 * 5. mt7996_dma_enable() - Configure and start DMA
 *
 * IMPORTANT: Rings are programmed AFTER dma_disable, while RST=0x30.
 * The RST is cleared inside mt7996_dma_enable -> mt7996_dma_start.
 */
int mt7996_dma_init(struct mt7996_dev *dev)
{
	struct mtk_wed_device *wed = &dev->mt76.mmio.wed;
	u32 rx_base;
	u32 hif1_ofs = 0;
	int ret;

	mt7996_dma_config(dev);

	mt76_dma_attach(&dev->mt76);

	if (dev->hif2)
		hif1_ofs = MT_WFDMA0_PCIE1(0) - MT_WFDMA0(0);

	mt7996_dma_disable(dev, true);

	/* init tx queue */
	ret = mt7996_init_tx_queues(&dev->phy,
				    MT_TXQ_ID(dev->mphy.band_idx),
				    MT7996_TX_RING_SIZE,
				    MT_TXQ_RING_BASE(0),
				    wed);
	if (ret)
		return ret;

	/* command to WM */
	ret = mt76_init_mcu_queue(&dev->mt76, MT_MCUQ_WM,
				  MT_MCUQ_ID(MT_MCUQ_WM),
				  MT7996_TX_MCU_RING_SIZE,
				  MT_MCUQ_RING_BASE(MT_MCUQ_WM));
	if (ret)
		return ret;

	/* command to WA */
	if (mt7996_has_wa(dev)) {
		ret = mt76_init_mcu_queue(&dev->mt76, MT_MCUQ_WA,
					  MT_MCUQ_ID(MT_MCUQ_WA),
					  MT7996_TX_MCU_RING_SIZE,
					  MT_MCUQ_RING_BASE(MT_MCUQ_WA));
		if (ret)
			return ret;
	}

	/* firmware download */
	ret = mt76_init_mcu_queue(&dev->mt76, MT_MCUQ_FWDL,
				  MT_MCUQ_ID(MT_MCUQ_FWDL),
				  MT7996_TX_FWDL_RING_SIZE,
				  MT_MCUQ_RING_BASE(MT_MCUQ_FWDL));
	if (ret)
		return ret;

	/* event from WM */
	ret = mt76_queue_alloc(dev, &dev->mt76.q_rx[MT_RXQ_MCU],
			       MT_RXQ_ID(MT_RXQ_MCU),
			       MT7996_RX_MCU_RING_SIZE,
			       MT7996_RX_MCU_BUF_SIZE,
			       MT_RXQ_RING_BASE(MT_RXQ_MCU));
	if (ret)
		return ret;

	/* ... (more queue allocations) ... */

	ret = mt76_init_queues(dev, mt76_dma_rx_poll);
	if (ret < 0)
		return ret;

	netif_napi_add_tx(dev->mt76.tx_napi_dev, &dev->mt76.tx_napi,
			  mt7996_poll_tx);
	napi_enable(&dev->mt76.tx_napi);

	mt7996_dma_enable(dev, false);

	return 0;
}

void mt7996_dma_cleanup(struct mt7996_dev *dev)
{
	mt7996_dma_disable(dev, true);
	mt76_dma_cleanup(&dev->mt76);
}
